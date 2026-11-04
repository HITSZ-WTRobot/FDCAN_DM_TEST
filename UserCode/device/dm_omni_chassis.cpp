#include "device/dm_omni_chassis.hpp"
#include "isr_lock.h"
#include "fdcan.h"

namespace device
{
namespace
{
motors::DMMotor::Config motorConfig(bsp::can::Can& can, std::size_t wheel,
                                   const DMOmniChassis::Config& config)
{
    return {
        .can = can,
        .id0 = DMOmniChassis::MotorIds[wheel],
        .type = motors::DMMotor::Type::S3519,
        .mode = motors::DMMotor::Mode::Vel,
        .pos_max_rad = 12.5f,
        .vel_max_rad = 200.0f,
        .tor_max = 10.0f,
        .auto_zero = false,
        .reverse = config.reverse[wheel],
        .reduction_rate = 1.0f, // User confirmed direct drive after the internal reducer.
        .can_fd_brs = true
    };
}
constexpr controllers::MotorVelController::Config WheelControl{
    .ctrl_mode = controllers::ControlMode::InternalVel,
    .internal_set_ratio = 1 // Future Controller must provide its explicit hardware update schedule.
};
}

DMOmniChassis::DMOmniChassis(bsp::can::Can& can, const Config& config) :
    can_(can),
    motors_{motors::DMMotor{motorConfig(can, 0, config)},
            motors::DMMotor{motorConfig(can, 1, config)},
            motors::DMMotor{motorConfig(can, 2, config)},
            motors::DMMotor{motorConfig(can, 3, config)}},
    wheel_control_{controllers::MotorVelController{&motors_[0], WheelControl},
                   controllers::MotorVelController{&motors_[1], WheelControl},
                   controllers::MotorVelController{&motors_[2], WheelControl},
                   controllers::MotorVelController{&motors_[3], WheelControl}},
    motion_({.wheel_radius = WheelRadiusMm,
             .wheel_distance_x = WheelDistanceXmm,
             .wheel_distance_y = WheelDistanceYmm,
             .wheel_front_right = &wheel_control_[0],
             .wheel_front_left = &wheel_control_[1],
             .wheel_rear_left = &wheel_control_[2],
             .wheel_rear_right = &wheel_control_[3]})
{
}

DMOmniChassis::Startup DMOmniChassis::start()
{
    if (startup_ != Startup::NotStarted)
        return startup_; // No implicit restart/recovery.
    if (motors::DMMotor::CAN_FilterInit(can_, 0) != bsp::can::Status::Ok ||
        can_.configure_filter(bsp::can::GlobalFilter{
            .reject_standard_remote = true, .reject_extended_remote = true}) != bsp::can::Status::Ok)
        return startup_ = Startup::FilterFailed;
    const auto sample_mtq = hfdcan2.Init.DataPrescaler * (1U + hfdcan2.Init.DataTimeSeg1);
    if (can_.configure_tx_delay_compensation({true, sample_mtq, 0}) != bsp::can::Status::Ok)
        return startup_ = Startup::DelayCompensationFailed;
    if (can_.start() != bsp::can::Status::Ok)
        return startup_ = Startup::CanStartFailed;
    return startup_ = Startup::Ready;
}

bool DMOmniChassis::pollDisabled()
{
    if (startup_ != Startup::Ready || motion_.enabled())
        return false;
    const auto before = can_.counters().tx_submitted_frames;
    for (auto& motor : motors_)
        motor.disable(); // Explicit hardware disable; releasing a controller alone is insufficient.
    return can_.counters().tx_submitted_frames - before == WheelCount;
}

DMOmniChassis::Feedback DMOmniChassis::feedback()
{
    ISRGuard guard;
    Feedback result{};
    for (std::size_t i = 0; i < WheelCount; ++i)
    {
        const auto& motor = motors_[i];
        result.wheels[i] = {motor.isConnected(), motor.state(), motor.getAngle(),
                            motor.getVelocity(), motor.getTorque()};
    }
    result.velocity_in_body = motion_.forwardGetVelocity();
    return result;
}
} // namespace device
