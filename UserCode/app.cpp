#include "cmsis_os2.h"
#include "config.hpp"
#include "communication/host_control.hpp"
#include "memory.hpp"
#include "usart.h"
#include "device/dm_omni_chassis.hpp"
#include "device/dm_position_test.hpp"
#include "device/m2006_velocity.hpp"
#include "fdcan.h"
#include "static_object.hpp"
#include "tim.h"
#include "watchdog.hpp"

bsp::can::Can* dm_can = nullptr;
device::DMPositionTest* dm_motors[6]{}; // 顺序、ID 和控制参数见 config.hpp。

bsp::can::Can* chassis_can = nullptr;
device::DMOmniChassis* dm_chassis = nullptr;
device::DMOmniChassis::Feedback chassis_feedback{};
volatile device::DMOmniChassis::Startup chassis_startup{device::DMOmniChassis::Startup::NotStarted};
volatile bool chassis_poll_ok{false}; // 仅表示硬件提交，不代表已确认物理失能。

bsp::can::Can* m2006_can = nullptr;
device::M2006Velocity* m2006_motors[2]{}; // 与 dm_motors 一样，均为独立的常驻对象。
volatile std::uint32_t m2006_tx_failures{0};

// 设备参数位于 config.hpp。启动过程只构造对象并启动
// 已配置的外设；不会自动使能或写入硬件参数。
// 操作、安全流程及串口协议见 doc/OPERATION.md 和 doc/SERIAL_PROTOCOL.md。

enum class Startup : std::uint32_t
{
    NotStarted, Ready, TaskFailed, FilterFailed, ReceiveCallbackFailed,
    DelayCompensationFailed, CanStartFailed, TimerCallbackFailed, TimerStartFailed, SerialStartFailed
};
volatile Startup dm_startup{Startup::NotStarted};
volatile std::uint32_t dm_rx_frames{0};
volatile std::uint32_t dm_tx_submitted_frames{0}; // 表示硬件提交，不代表电机执行。
volatile std::uint32_t dm_tx_failures{0};
volatile bool host_link_timeout{false};
volatile std::uint32_t host_crc_errors{0}, host_rx_dropped_bytes{0}, host_uart_errors{0}, host_tx_errors{0};

// 现有固定存储包显式执行 placement new。对象常驻，
// 不会从工程的小容量 C/C++ 堆分配内存，也不会在 ISR 中分配内存。
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
static memory::StaticObject<device::M2006Velocity> m2006_motor_storage[2];

static void motor_init()
{
    dm_can = INIT_OBJECT(can_storage, hfdcan1);
    for (unsigned i = 0; i < 6; ++i)
    {
        const auto& parameters = config::dm_motor_parameters[i];
        const motors::DMMotor::Config motor_config{
            .can = *dm_can,
            .id0 = parameters.can_id,
            .type = parameters.type,
            .mode = motors::DMMotor::Mode::MIT,
            .pos_max_rad = parameters.pos_max_rad,
            .vel_max_rad = parameters.vel_max_rad_s,
            .tor_max = parameters.torque_max_nm,
            .auto_zero = false, // 设备等待真正更新且表明静止的反馈。
            .reverse = parameters.reverse,
            // reduction_rate仅表示外接减速；S3519反馈归一化仍由现有Type路径负责。
            // MIT位置参考的逆向坐标换算在设备层完成，不修改公共驱动。
            .reduction_rate = parameters.reduction_rate,
            .can_fd_brs = true
        };
        const auto kind = parameters.type == motors::DMMotor::Type::S3519 ?
                          device::PositionKind::LinearActuator : device::PositionKind::Joint;
        dm_motors[i] = INIT_OBJECT(motor_storage[i], motor_config, config::dm_control[i], kind);
        const auto& limits = kind == device::PositionKind::LinearActuator ?
                             config::linear_motion_dynamics : config::joint_motion_limits;
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
        if (id == config::dm_motor_parameters[i].can_id &&
            frame.header.id == motors::DMMotor::MasterIDOffset + config::dm_motor_parameters[i].can_id)
        {
            dm_motors[i]->feedbackReceived();
            break;
        }
    }
}


static bool can_init()
{
    // 上游 DM 驱动负责其标准掩码过滤器（0x028/0x7F8），覆盖
    // 合法回复 0x029..0x02F。实例回调拒绝不匹配的 ID。
    if (motors::DMMotor::CAN_FilterInit(*dm_can, 0) != bsp::can::Status::Ok ||
        dm_can->configure_filter(bsp::can::GlobalFilter{
            .reject_standard_remote = true, .reject_extended_remote = true}) != bsp::can::Status::Ok ||
        motors::DJIMotor::CAN_FilterInit(*m2006_can, 0) != bsp::can::Status::Ok ||
        m2006_can->configure_filter(bsp::can::GlobalFilter{
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
    // FD+BRS 速率为 5 Mbps：SSP 偏移遵循已配置的数据采样点。
    // 驱动/HAL 测量 TXD->RXD 环路延迟；不猜测收发器延迟。
    const auto sample_mtq = hfdcan1.Init.DataPrescaler * (1U + hfdcan1.Init.DataTimeSeg1);
    if (dm_can->configure_tx_delay_compensation({true, sample_mtq, 0}) != bsp::can::Status::Ok)
    {
        dm_startup = Startup::DelayCompensationFailed;
        return false;
    }
    if (dm_can->start() != bsp::can::Status::Ok || m2006_can->start() != bsp::can::Status::Ok)
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
}

static void motorFeedbackUpdate(TIM_HandleTypeDef*) // TIM3, 500 Hz.
{
    // 唯一负责方：默认看门狗预算为 10 个节拍，此处约为 20 ms。
    service::Watchdog::EatAll();
    host_control->linkTick(HAL_GetTick()); // 上位机 500ms 超时处理无需等待任务。
    for (auto* motor : dm_motors)
        motor->feedbackUpdate();
    chassis_poll_ok = dm_chassis->update(); // FDCAN2 周期速度/失能的唯一负责方，500Hz。
    for (auto* motor : m2006_motors)
        motor->feedbackUpdate(); // TIM3 500Hz：仅观察速度反馈和故障。
}

static void motorMitSendUpdate(TIM_HandleTypeDef*) // TIM6，2 kHz；每台电机 1 kHz。
{
    // 唯一发送方：A=[0,1,2] (0A/0B/0C)，B=[3,4,5] (0D/0E/0F)。
    // 交替使用硬件节拍，使两组提交间隔为 0.5 ms，无需忙等待。
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
    if (next_group == 0) // 每两个 TIM6 节拍：更新两个速度环，共用一帧发送。
    {
        for (auto* motor : m2006_motors)
            motor->velocityUpdate();
        if (motors::DJIMotor::SendIqCommand(*m2006_can, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4)
            != bsp::can::SendResult::Submitted)
        {
            m2006_tx_failures = m2006_tx_failures + 1;
            for (auto* motor : m2006_motors)
                motor->transmitFailed(); // 缓存零值；不立即重试。
        }
    }
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
    // 初始化使用本任务的 4096 字节栈，而非 Init 的 512 字节栈。
    // 启动三条总线或定时器前，先构造并注册全部十二台电机。
    motor_init();
    chassis_can = INIT_OBJECT(chassis_can_storage, hfdcan2);
    dm_chassis = INIT_OBJECT(chassis_storage, *chassis_can, config::chassis);
    m2006_can = INIT_OBJECT(m2006_can_storage, hfdcan3);
    for (unsigned i = 0; i < 2; ++i)
    {
        const auto& parameters = config::m2006_motor_parameters[i];
        const motors::DJIMotor::Config motor_config{
            .can=*m2006_can, .type=motors::DJIMotor::Type::M2006_C610,
            .id1=parameters.id, .auto_zero=false,
            .reverse=parameters.reverse, .reduction_rate=parameters.reduction_rate
        };
        m2006_motors[i] = INIT_OBJECT(m2006_motor_storage[i], motor_config, config::m2006_control[i]);
    }
    host_uart = INIT_OBJECT(host_uart_storage, huart1, host_rx_dma, host_tx_dma);
    host_control = INIT_OBJECT(host_control_storage, *host_uart, dm_motors, *dm_chassis, m2006_motors);
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
    if (!can_init() || !tim_init())
    {
        osThreadExit();
        return;
    }
    dm_startup = Startup::Ready;
    for (;;)
    {
        host_control->poll(HAL_GetTick()); // 串口和 Ozone 命令共用一个消费者。
        host_link_timeout = host_control->timedOut();
        host_crc_errors = host_control->crcErrors();
        host_rx_dropped_bytes = host_uart->rxDrops();
        host_uart_errors = host_uart->uartErrors();
        host_tx_errors = host_uart->txErrors();
        chassis_feedback = dm_chassis->feedback();
        for (auto* motor : m2006_motors)
            motor->service();
        osDelay(1); // 仅服务协议；电机定时仍由硬件驱动。
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
