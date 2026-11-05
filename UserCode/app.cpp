#include "cmsis_os2.h"
#include "config.hpp"
#include "communication/host_control.hpp"
#include "memory.hpp"
#include "usart.h"
#include "device/dm_omni_chassis.hpp"
#include "device/dm_position_test.hpp"
#include "device/m2006_position.hpp"
#include "fdcan.h"
#include "static_object.hpp"
#include "tim.h"
#include "watchdog.hpp"

bsp::can::Can* dm_can = nullptr;
device::DMPositionTest* dm_motors[6]{}; // Order/IDs and control values: config.hpp.

bsp::can::Can* chassis_can = nullptr;
device::DMOmniChassis* dm_chassis = nullptr;
device::DMOmniChassis::Feedback chassis_feedback{};
volatile device::DMOmniChassis::Startup chassis_startup{device::DMOmniChassis::Startup::NotStarted};
volatile bool chassis_poll_ok{false}; // Hardware submission only, not physical disable confirmation.

bsp::can::Can* m2006_can = nullptr;
device::M2006Position* m2006_motors[2]{}; // Independent permanent objects, like dm_motors.
volatile std::uint32_t m2006_tx_failures{0};

// Device parameters live in config.hpp. Startup only constructs objects and starts
// already-configured peripherals; no automatic enable or hardware parameter writes.
// Operation/safety procedure and serial protocol are documented in README.md.

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
static memory::StaticObject<device::M2006Position> m2006_motor_storage[2];

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
            .auto_zero = false, // Device waits for genuinely new, stationary feedback.
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
    // Upstream DM driver owns its standard mask filter (0x028/0x7F8), covering
    // legal replies 0x029..0x02F. Instance callbacks reject nonmatching IDs.
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
    // FD+BRS at 5 Mbps: SSP offset follows the configured data sample point.
    // Driver/HAL measure the TXD->RXD loop delay; no guessed transceiver delay.
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
    for (auto* motor : m2006_motors)
        motor->profileUpdate(); // TIM7 100Hz: each independent trajectory/velocity feedforward.
}

static void motorFeedbackUpdate(TIM_HandleTypeDef*) // TIM3, 500 Hz.
{
    // One owner: default watchdog budget 10 ticks -> approximately 20 ms here.
    service::Watchdog::EatAll();
    host_control->linkTick(HAL_GetTick()); // Host 500ms timeout does not wait for the task.
    for (auto* motor : dm_motors)
        motor->feedbackUpdate();
    chassis_poll_ok = dm_chassis->update(); // FDCAN2 periodic velocity/disable owner,500Hz.
    for (auto* motor : m2006_motors)
        motor->positionUpdate(); // TIM3 500Hz: each independent position PD.
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
    if (next_group == 0) // Every second TIM6 tick: both velocity loops, one shared frame.
    {
        for (auto* motor : m2006_motors)
            motor->velocityUpdate();
        if (motors::DJIMotor::SendIqCommand(*m2006_can, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4)
            != bsp::can::SendResult::Submitted)
        {
            m2006_tx_failures = m2006_tx_failures + 1;
            for (auto* motor : m2006_motors)
                motor->transmitFailed(); // Cache zero; no immediate retry.
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
    // Initialization runs on this 4096-byte task stack, not the 512-byte Init stack.
    // Construct/register all twelve motors before starting the three buses or timers.
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
        host_control->poll(HAL_GetTick()); // Serial and Ozone commands share one consumer.
        host_link_timeout = host_control->timedOut();
        host_crc_errors = host_control->crcErrors();
        host_rx_dropped_bytes = host_uart->rxDrops();
        host_uart_errors = host_uart->uartErrors();
        host_tx_errors = host_uart->txErrors();
        chassis_feedback = dm_chassis->feedback();
        for (auto* motor : m2006_motors)
            motor->service();
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
