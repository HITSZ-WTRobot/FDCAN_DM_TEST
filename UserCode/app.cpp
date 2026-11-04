#include "cmsis_os2.h"
#include "communication/host_control.hpp"
#include "memory.hpp"
#include "usart.h"
#include "device/dm_omni_chassis.hpp"
#include "device/dm_position_test.hpp"
#include "device/m2006_group.hpp"
#include "fdcan.h"
#include "static_object.hpp"
#include "tim.h"
#include "watchdog.hpp"

// Fixed MIT packet parameters, copied at construction; no MCU position/velocity PID.
// Kp=5.0, Kd=0.5, t_ff=0 are low-gain starting values, not validated loaded tuning.
// Feedback stationarity thresholds account for each image's velocity quantization.
device::ControlParameters dm_control[6]{
    {.stationary_rpm = 1.1f}, // [0] 右前伸缩：VMAX=200，反馈一步约0.933 rpm。
    {.stationary_rpm = 1.1f}, // [1] 中伸缩。
    {.stationary_rpm = 1.1f}, // [2] 后伸缩。
    {.stationary_rpm = 0.1f}, // [3] 左前旋转：VMAX=10，反馈一步约0.0467 rpm。
    {.stationary_rpm = 1.1f}, // [4] 左前伸缩。
    {.stationary_rpm = 0.1f}  // [5] 右前旋转。
};

bsp::can::Can* dm_can = nullptr;
device::DMPositionTest* dm_motors[6]{}; // 索引及部位见下方表；每台Master ID=0x20+CAN ID。

// FDCAN2 is independent of the six position-test motors on FDCAN1.
// User confirmed: 127mm wheels, 435mm square, direct drive, initial positive directions.
device::DMOmniChassis::Config chassis_config{}; // reverse[FR, FL, RL, RR], copied at construction.
bsp::can::Can* chassis_can = nullptr;
device::DMOmniChassis* dm_chassis = nullptr;
device::DMOmniChassis::Feedback chassis_feedback{};
volatile device::DMOmniChassis::Startup chassis_startup{device::DMOmniChassis::Startup::NotStarted};
volatile bool chassis_poll_ok{false}; // Hardware submission only, not physical disable confirmation.

// User image: discrete gains are used verbatim at the specified 1k/500/100Hz rates.
// C610 protocol current units: 8000 means 8A torque-current command, not input current.
device::M2006Position::Config m2006_control[2]{
    {.velocity_pid = {9.0f, 0.1f, 0.0f, 8000.0f},
     .position_pd = {5.0f, 4000.0f, 400.0f}, .profile = {1800.0f, 1200.0f, 3000.0f}},
    {.velocity_pid = {9.0f, 0.1f, 0.0f, 8000.0f},
     .position_pd = {5.0f, 4000.0f, 400.0f}, .profile = {1800.0f, 1200.0f, 3000.0f}}
};
bsp::can::Can* m2006_can = nullptr;
device::M2006Group* m2006_group = nullptr;

struct MotorParameters
{
    std::uint8_t can_id;
    motors::DMMotor::Type type;
    float pos_max_rad;
    float vel_max_rad_s;
    float torque_max_nm;
};

// 来源：参考资料/参考资料/电机参数；不含四个底盘电机。
// PMAX/VMAX/TMAX是电机端协议映射量程，不是机构运动限制或外部PID增益。
static constexpr std::uint32_t MotorMasterIdBase = motors::DMMotor::MasterIDOffset;
static constexpr MotorParameters motor_parameters[6]{
    {0x0A, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // [0] 右前伸缩电机，DM-S3519
    {0x0B, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // [1] 中伸缩电机，DM-S3519
    {0x0C, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // [2] 后伸缩电机，DM-S3519
    {0x0D, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}, // [3] 左前旋转电机，DM-J4340P
    {0x0E, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // [4] 左前伸缩电机，DM-S3519
    {0x0F, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}  // [5] 右前旋转电机，DM-J4340P
};

// Joint zero always recenters the +/-180deg interval. Linear travel has no software
// min/max; only dynamics and the original absolute MIT encoding bounds are retained.
static constexpr device::MotionLimits joint_motion_limits{
    -180.0f, 180.0f, 10.0f, 20.0f, 100.0f
};
static constexpr device::MotionLimits linear_motion_dynamics{
    .max_velocity_deg_s = 10.0f, .max_acceleration_deg_s2 = 20.0f, .max_jerk_deg_s3 = 100.0f
}; // Equivalent to 6.2833mm/s, 12.5667mm/s^2, 62.8333mm/s^3; no travel interval.

/* Ozone 操作（MIT直接跟踪，未在MCU建立外部位置环或速度环）：
 * 实物须先设为MIT；CAN ID、型号和量程按motor_parameters。
 * 每台Master ID=0x20+CAN ID，即0x2A~0x2F；须由操作者同步设置电机端。
 * 图中CAN Timeout均为0，不能依赖电机端超时自动停机；本程序不改写该参数。
 * 仲裁1M/数据5M、FD+BRS保持已确认配置，实板收发与稳定性仍须验证。
 * dm_control在构造时复制；改完重启，或在motor_init之前通过Ozone填写。
 * kp/kd是每条MIT帧发送的增益，不是速度KP/KI；初值5.0/0.5，torque_ff_nm=0。
 * 电机执行内部MIT控制。前馈只是附加力矩，不是总输出力矩限制；当前位置误差
 * 仍会产生保持力矩。初值不保证带载稳定或重力补偿，须支撑机构并逐台微调。
 * finish_deg=0.3为到位观察阈值；stationary_rpm按两种协议速度分辨率设置，
 * 均不是机械保护参数。kp/kd非法或为0时拒绝使能；动态约束见上方两组配置。
 *
 * 1. dm_startup=Ready后保持失能；50次新的失能且静止反馈，在两倍位置协议
 *    分辨率内稳定后建立本次启动的软件零点。驱动内部绝对坐标不清零。
 *    S3519依用户实测保留减速前位置口径：反馈由库除19.203，发送位置参考
 *    在接入层换回原口径；速度仍按库的输出轴反馈契约，不再叠加内置减速比。
 * 2. 首次稳定反馈后软件归零：关节以新原点建立±180°；伸缩不设软件行程限位。
 *    速度10°/s、加速度20°/s²、jerk100°/s³保持原值。伸缩按226.2mm/输出轴圈换算。
 *    只有完整关节范围或当前伸缩零点可编码、动态参数有效时才提交归零；失败保留旧配置。
 *    首次失败时zeroed/limits_configured仍为false。伸缩每个目标仍须满足绝对PMAX。
 * 3. 等command=None、result=Ok，再写Enable(2)。先预装当前位置保持参考，
 *    再请求使能；等待connected=true、enable_pending=false、enabled=true、fault=false。
 * 4. 关节先填target_deg再写Move(4)。伸缩使用moveToMm(绝对毫米位置)软件接口，
 *    linearFeedback()返回毫米状态；原始Move(4)不接受伸缩电机，避免单位误用。
 *    运动中Move返回Busy，不替换原轨迹。
 *    任务只求解新曲线；TIM7以100Hz推进位置/速度参考；TIM3以500Hz处理反馈、
 *    到位、零点与故障；TIM6以2kHz轮换发送两组三台，每台1kHz。
 *    使能、失能和故障停机也走各自发送时隙；实际线上时刻仍受CAN仲裁/重发影响。
 * 5. 正常到位看motion_complete；busy=false也可能是Stop/Disable/故障。
 *    Stop(5)取消曲线并保持当前位置，不是S曲线减速停车；Disable(3)请求失能。
 *    失能确认须connected=true且新的motor_state=0，断线不能证明实际已停。
 * 6. setSoftwareZero()/SetZero(1)仅在失能、静止且反馈有效时接受，不写电机Flash；
 *    关节每次重新建立±180°，伸缩只改变毫米原点。MIT参考仍保持绝对物理坐标。
 * 7. commanded_torque_ff_nm只表示MIT附加前馈；实际总力矩看torque_nm反馈。
 *    TX统计只代表提交到硬件FIFO，不代表执行。暂停CPU/单步前先失能并支撑机构；
 *    软件保护不代替机械支撑、硬件急停或电机超时设置。用户已确认烧录测试可行，
 *    工具未直接操作目标；未提供的负载/工况和安全性不作扩大保证。
 */

enum class Startup : std::uint32_t
{
    NotStarted, Ready, TaskFailed, FilterFailed, ReceiveCallbackFailed,
    DelayCompensationFailed, CanStartFailed, TimerCallbackFailed, TimerStartFailed, SerialStartFailed
};
volatile Startup dm_startup{Startup::NotStarted};
volatile std::uint32_t dm_rx_frames{0};
volatile std::uint32_t dm_tx_submitted_frames{0}; // Hardware submission, not motor execution.
volatile std::uint32_t dm_tx_failures{0};
volatile bool host_link_timeout{false};
volatile std::uint32_t host_crc_errors{0}, host_rx_dropped_bytes{0}, host_uart_errors{0}, host_tx_errors{0};

// Existing fixed-storage package performs explicit placement new. Objects are
// permanent and never allocate from the project's small C/C++ heap or an ISR.
static memory::StaticObject<bsp::can::Can> can_storage;
static memory::StaticObject<device::DMPositionTest> motor_storage[6];
static MEM_RAM_AXI memory::DMABuffer<512> host_rx_dma;
static MEM_RAM_AXI memory::DMABuffer<64> host_tx_dma;
static memory::StaticObject<host::Uart> host_uart_storage;
static memory::StaticObject<host::Control> host_control_storage;
static host::Control* host_control = nullptr;
static host::Uart* host_uart = nullptr;
static memory::StaticObject<bsp::can::Can> chassis_can_storage;
static memory::StaticObject<device::DMOmniChassis> chassis_storage;
static memory::StaticObject<bsp::can::Can> m2006_can_storage;
static memory::StaticObject<device::M2006Group> m2006_group_storage;

static void motor_init()
{
    dm_can = INIT_OBJECT(can_storage, hfdcan1);
    for (unsigned i = 0; i < 6; ++i)
    {
        const auto& parameters = motor_parameters[i];
        const motors::DMMotor::Config config{
            .can = *dm_can,
            .id0 = parameters.can_id,
            .type = parameters.type,
            .mode = motors::DMMotor::Mode::MIT,
            .pos_max_rad = parameters.pos_max_rad,
            .vel_max_rad = parameters.vel_max_rad_s,
            .tor_max = parameters.torque_max_nm,
            .auto_zero = false, // Device waits for genuinely new, stationary feedback.
            .reverse = false,
            // reduction_rate仅表示外接减速；S3519反馈归一化仍由现有Type路径负责。
            // MIT位置参考的逆向坐标换算在设备层完成，不修改公共驱动。
            .reduction_rate = 1.0f,
            .can_fd_brs = true
        };
        const auto kind = parameters.type == motors::DMMotor::Type::S3519 ?
                          device::PositionKind::LinearActuator : device::PositionKind::Joint;
        dm_motors[i] = INIT_OBJECT(motor_storage[i], config, dm_control[i], kind);
        const auto& limits = kind == device::PositionKind::LinearActuator ?
                             linear_motion_dynamics : joint_motion_limits;
        dm_motors[i]->min_deg = limits.min_deg;
        dm_motors[i]->max_deg = limits.max_deg;
        dm_motors[i]->max_velocity_deg_s = limits.max_velocity_deg_s;
        dm_motors[i]->max_acceleration_deg_s2 = limits.max_acceleration_deg_s2;
        dm_motors[i]->max_jerk_deg_s3 = limits.max_jerk_deg_s3;
    }
}

static void can_receive(const bsp::can::FrameView& frame)
{
    if (frame.header.id_type != bsp::can::IdType::Standard ||
        frame.header.type != bsp::can::FrameType::Data || frame.data.size() != 8)
        return;
    const unsigned id = frame.data[0] & 0x0f;
    for (unsigned i = 0; i < 6; ++i)
    {
        if (id == motor_parameters[i].can_id &&
            frame.header.id == MotorMasterIdBase + motor_parameters[i].can_id)
        {
            dm_motors[i]->feedbackReceived();
            break;
        }
    }
}


static bool can_init()
{
    // Upstream DM driver owns its standard mask filter (0x028/0x7F8), covering
    // legal replies 0x029..0x02F. Instance callbacks reject nonmatching IDs.
    if (motors::DMMotor::CAN_FilterInit(*dm_can, 0) != bsp::can::Status::Ok ||
        dm_can->configure_filter(bsp::can::GlobalFilter{
            .reject_standard_remote = true, .reject_extended_remote = true}) != bsp::can::Status::Ok)
    {
        dm_startup = Startup::FilterFailed;
        return false;
    }
    if (dm_can->register_callback(can_receive) != bsp::can::Status::Ok)
    {
        dm_startup = Startup::ReceiveCallbackFailed;
        return false;
    }
    // FD+BRS at 5 Mbps: SSP offset follows the configured data sample point.
    // Driver/HAL measure the TXD->RXD loop delay; no guessed transceiver delay.
    const auto sample_mtq = hfdcan1.Init.DataPrescaler * (1U + hfdcan1.Init.DataTimeSeg1);
    if (dm_can->configure_tx_delay_compensation({true, sample_mtq, 0}) != bsp::can::Status::Ok)
    {
        dm_startup = Startup::DelayCompensationFailed;
        return false;
    }
    if (dm_can->start() != bsp::can::Status::Ok)
    {
        dm_startup = Startup::CanStartFailed;
        return false;
    }
    return true;
}

static void motorProfileUpdate(TIM_HandleTypeDef*) // TIM7, 100 Hz.
{
    for (auto* motor : dm_motors)
        motor->profileUpdate();
    m2006_group->profileUpdate(); // TIM7 100Hz: trajectory velocity feedforward.
}

static void motorFeedbackUpdate(TIM_HandleTypeDef*) // TIM3, 500 Hz.
{
    // One owner: default watchdog budget 10 ticks -> approximately 20 ms here.
    service::Watchdog::EatAll();
    host_control->linkTick(HAL_GetTick()); // Host 500ms timeout does not wait for the task.
    for (auto* motor : dm_motors)
        motor->feedbackUpdate();
    chassis_poll_ok = dm_chassis->update(); // FDCAN2 periodic velocity/disable owner,500Hz.
    m2006_group->positionUpdate(); // TIM3 500Hz: position error compensation.
}

static void motorMitSendUpdate(TIM_HandleTypeDef*) // TIM6, 2 kHz; each motor 1 kHz.
{
    // Unique TX owner: A=[0,1,2] (0A/0B/0C), B=[3,4,5] (0D/0E/0F).
    // Alternating hardware ticks separate submissions by 0.5 ms without busy-waits.
    static unsigned next_group = 0;
    const unsigned first = next_group * 3U;
    next_group ^= 1U;
    for (unsigned i = first; i < first + 3U; ++i)
    {
        auto* motor = dm_motors[i];
        const auto before = dm_can->counters().tx_submitted_frames;
        motor->sendUpdate();
        if (dm_can->counters().tx_submitted_frames == before)
        {
            dm_tx_failures = dm_tx_failures + 1;
            motor->transmitFailed();
        }
    }
    const auto traffic = dm_can->counters();
    dm_rx_frames = traffic.rx_frames;
    dm_tx_submitted_frames = traffic.tx_submitted_frames;
    m2006_group->transmitTick(); // Device owns /2 cadence, 1kHz speed PID and grouped current output.
    
}

static bool tim_init()
{
    if (HAL_TIM_RegisterCallback(&htim7, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorProfileUpdate) != HAL_OK ||
        HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorFeedbackUpdate) != HAL_OK ||
        HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorMitSendUpdate) != HAL_OK)
    {
        dm_startup = Startup::TimerCallbackFailed;
        return false;
    }
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK || HAL_TIM_Base_Start_IT(&htim3) != HAL_OK ||
        HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        HAL_TIM_Base_Stop_IT(&htim7);
        HAL_TIM_Base_Stop_IT(&htim3);
        HAL_TIM_Base_Stop_IT(&htim6);
        dm_startup = Startup::TimerStartFailed;
        return false;
    }
    return true;
}

static void motorCommandUpdate(void*)
{
    // Initialization runs on this 4096-byte task stack, not the 512-byte Init stack.
    // Construct/register all twelve motors before starting the three buses or timers.
    motor_init();
    chassis_can = INIT_OBJECT(chassis_can_storage, hfdcan2);
    dm_chassis = INIT_OBJECT(chassis_storage, *chassis_can, chassis_config);
    m2006_can = INIT_OBJECT(m2006_can_storage, hfdcan3);
    m2006_group = INIT_OBJECT(m2006_group_storage, *m2006_can, m2006_control);
    host_uart = INIT_OBJECT(host_uart_storage, huart1, host_rx_dma, host_tx_dma);
    host_control = INIT_OBJECT(host_control_storage, *host_uart, dm_motors, *dm_chassis, *m2006_group);
    chassis_startup = dm_chassis->start();
    if (chassis_startup != device::DMOmniChassis::Startup::Ready)
    {
        osThreadExit();
        return;
    }
    if (!host_uart->start())
    {
        dm_startup = Startup::SerialStartFailed;
        osThreadExit();
        return;
    }
    if (!can_init() || m2006_group->start() != device::M2006Group::Startup::Ready || !tim_init())
    {
        osThreadExit();
        return;
    }
    dm_startup = Startup::Ready;
    for (;;)
    {
        host_control->poll(HAL_GetTick()); // Serial and Ozone commands share one consumer.
        host_link_timeout = host_control->timedOut();
        host_crc_errors = host_control->crcErrors();
        host_rx_dropped_bytes = host_uart->rxDrops();
        host_uart_errors = host_uart->uartErrors();
        host_tx_errors = host_uart->txErrors();
        chassis_feedback = dm_chassis->feedback();
        m2006_group->service();
        osDelay(1); // Protocol service only; motor timing remains hardware-driven.
    }
}

extern "C" void Init(void*)
{
    osThreadAttr_t attributes{};
    attributes.name = "dm_command";
    attributes.priority = osPriorityNormal;
    attributes.stack_size = 4096;
    if (osThreadNew(motorCommandUpdate, nullptr, &attributes) == nullptr)
        dm_startup = Startup::TaskFailed;
    osThreadExit();
}
