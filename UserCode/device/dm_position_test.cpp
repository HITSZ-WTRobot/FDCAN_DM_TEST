#include "device/dm_position_test.hpp"
#include "isr_lock.h"
#include <algorithm>
#include <cmath>

namespace device
{
namespace
{
constexpr float PositionLsbDeg = 12.5f * 180.0f / (32767.0f * 3.14159265358979323846f);
constexpr float ProtocolMaxVelocityDegS = 30.0f * 180.0f / 3.14159265358979323846f;
constexpr float ProtocolMaxTorque = 10.0f;
constexpr unsigned StableSamples = 50;

bool nonnegative(float value) { return std::isfinite(value) && value >= 0.0f; }
bool positive(float value) { return std::isfinite(value) && value > 0.0f; }
} // namespace

DMPositionTest::DMPositionTest(const motors::DMMotor::Config& motor_config,
                               const ControlParameters& control) :
    motor_(motor_config),
    controller_(&motor_, {control.velocity_pid, controllers::ControlMode::ExternalPID}),
    trajectory_(&controller_, {0.0f, 0.0f, 0.0f}, control.position_pd, control.finish_deg),
    control_(control)
{
}

bool DMPositionTest::validControl() const noexcept
{
    const auto& pid = control_.velocity_pid;
    const auto& pd = control_.position_pd;
    return positive(pid.Kp) && nonnegative(pid.Ki) && nonnegative(pid.Kd) &&
           positive(pid.abs_output_max) && pid.abs_output_max <= ProtocolMaxTorque &&
           positive(pd.Kp) && nonnegative(pd.Kd) && positive(pd.abs_output_max) &&
           positive(control_.finish_deg) && positive(control_.stationary_rpm);
}

bool DMPositionTest::validLimits(const MotionLimits& l) const noexcept
{
    return std::isfinite(l.min_deg) && std::isfinite(l.max_deg) && l.min_deg < l.max_deg &&
           positive(l.max_velocity_deg_s) && l.max_velocity_deg_s <= ProtocolMaxVelocityDegS &&
           positive(l.max_acceleration_deg_s2) && positive(l.max_jerk_deg_s3);
}

bool DMPositionTest::inRange(float position) const noexcept
{
    return std::isfinite(position) && position >= limits_.min_deg && position <= limits_.max_deg;
}

void DMPositionTest::feedbackReceived() noexcept
{
    feedback_count = feedback_count + 1;
}

void DMPositionTest::disableOutput(bool latch_fault)
{
    trajectory_.disable();
    controller_.getPID().reset();
    controller_.setRef(0.0f);
    motor_.disable();
    commanded_torque_nm = 0.0f;
    enable_requested_ = false;
    enable_pending = false;
    enable_wait_ticks_ = 0;
    busy = false;
    motion_complete = false;
    if (latch_fault)
        fault = true;
    // enabled is deliberately not cleared here: only feedback can confirm disable.
}

void DMPositionTest::observeFeedback()
{
    connected = motor_.isConnected();
    if (!connected)
    {
        stable_samples_ = 0;
        enabled = false; // No current enable confirmation; not proof of physical disable.
        if (ever_connected_ && !fault)
        {
            disableOutput(true);
            result = Result::Offline;
        }
        return;
    }
    ever_connected_ = true;
    const auto count = feedback_count;
    if (count == observed_feedback_count_)
        return;
    observed_feedback_count_ = count;
    position_deg = motor_.getAngle();
    velocity_rpm = motor_.getVelocity();
    torque_nm = motor_.getTorque();
    motor_state = static_cast<std::uint32_t>(motor_.state());
    enabled = motor_state == 1;
    if (!std::isfinite(position_deg) || !std::isfinite(velocity_rpm) ||
        !std::isfinite(torque_nm) || motor_state > 1)
    {
        stable_samples_ = 0;
        if (!fault)
            disableOutput(true);
        result = Result::MotorFault;
        return;
    }
    if (enable_pending && enabled)
        enable_pending = false;
    if (enable_requested_ && !enable_pending && !enabled)
    {
        disableOutput(true);
        result = Result::MotorFault;
        return;
    }
    if (motor_state != 0 || !positive(control_.stationary_rpm) ||
        std::fabs(velocity_rpm) > control_.stationary_rpm)
    {
        stable_samples_ = 0;
        return;
    }
    const float position = position_deg;
    if (stable_samples_ == 0)
    {
        stable_min_ = stable_max_ = position;
        stable_samples_ = 1;
    }
    else
    {
        const float low = std::min(stable_min_, position);
        const float high = std::max(stable_max_, position);
        if (high - low > 2.0f * PositionLsbDeg)
        {
            stable_min_ = stable_max_ = position;
            stable_samples_ = 1;
        }
        else
        {
            stable_min_ = low;
            stable_max_ = high;
            stable_samples_ = std::min(stable_samples_ + 1, StableSamples);
        }
    }
    if (!zeroed && stable_samples_ >= StableSamples && !command_active_)
        (void)setZero();
}

Result DMPositionTest::applyLimits()
{
    if (enable_requested_ || motor_.state() != motors::DMMotor::State::Disabled)
        return Result::Busy;
    const MotionLimits staged{min_deg, max_deg, max_velocity_deg_s,
                              max_acceleration_deg_s2, max_jerk_deg_s3};
    if (!validLimits(staged))
        return Result::InvalidParameters;
    // Configure limits only after establishing the coordinate frame.
    if (!zeroed)
        return Result::NotZeroed;
    limits_ = staged;
    trajectory_.setDefaultProfileConfig({staged.max_velocity_deg_s,
                                        staged.max_acceleration_deg_s2, staged.max_jerk_deg_s3});
    limits_configured = true;
    return Result::Ok;
}

Result DMPositionTest::setZero()
{
    if (!motor_.isConnected())
        return Result::Offline;
    if (enable_requested_ || motor_.state() != motors::DMMotor::State::Disabled)
        return Result::Busy;
    if (stable_samples_ < StableSamples)
        return Result::NotStationary;
    const float shift = motor_.getAngle();
    if (limits_configured)
    {
        MotionLimits translated = limits_;
        translated.min_deg -= shift;
        translated.max_deg -= shift;
        if (!validLimits(translated))
            return Result::InvalidParameters;
        limits_ = translated;
        min_deg = translated.min_deg;
        max_deg = translated.max_deg;
    }
    motor_.resetAngle(); // MCU software coordinate only; never writes the motor's saved zero.
    controller_.getPID().reset();
    trajectory_.stop();
    controller_.setRef(0.0f);
    position_deg = 0.0f;
    target_deg = 0.0f;
    accepted_target_deg = 0.0f;
    zeroed = true;
    busy = false;
    motion_complete = false;
    stable_samples_ = 0;
    return Result::Ok;
}

Result DMPositionTest::enableOutput()
{
    if (!motor_.isConnected())
        return Result::Offline;
    if (enable_requested_)
        return Result::Busy;
    if (motor_.state() != motors::DMMotor::State::Disabled)
        return Result::MotorFault;
    if (!zeroed)
        return Result::NotZeroed;
    if (!validControl() || !limits_configured)
        return Result::InvalidParameters;
    if (stable_samples_ < StableSamples || std::fabs(motor_.getVelocity()) > control_.stationary_rpm)
        return Result::NotStationary;
    if (!inRange(motor_.getAngle()))
        return Result::OutsideLimits;
    controller_.getPID().reset();
    motor_.setCurrent(0.0f); // Clear any stale MIT reference before requesting enable.
    if (!trajectory_.enable())
    {
        motor_.disable();
        return Result::NotEnabled;
    }
    // stop() alone does not update the controller reference or PD history.
    trajectory_.errorUpdate();
    controller_.setRef(0.0f);
    target_deg = motor_.getAngle();
    accepted_target_deg = target_deg;
    enable_requested_ = true;
    enable_pending = true;
    enable_wait_ticks_ = WATCHDOG_SNACKS;
    busy = false;
    motion_complete = false;
    fault = false;
    return Result::Ok; // Accepted; enable_pending waits for an enabled feedback frame.
}

Result DMPositionTest::move(float target)
{
    {
        ISRGuard guard;
        if (!motor_.isConnected())
            return Result::Offline;
        if (!enable_requested_ || enable_pending || fault || !enabled)
            return Result::NotEnabled;
        if (busy)
            return Result::Busy;
        if (!inRange(target) || !inRange(motor_.getAngle()))
            return Result::OutsideLimits;
        if (std::fabs(motor_.getVelocity()) > control_.stationary_rpm)
            return Result::NotStationary;
        trajectory_.stop();
    }
    // Existing library solves the curve outside the interrupt critical section.
    const bool planned = trajectory_.setTarget(target);
    ISRGuard guard;
    if (fault || !enable_requested_ || !motor_.isConnected())
    {
        trajectory_.disable();
        return Result::Offline;
    }
    if (!planned)
        return Result::PlanningFailed;
    target_deg = target;
    accepted_target_deg = target;
    busy = true;
    motion_complete = false;
    return Result::Ok;
}

void DMPositionTest::commandUpdate()
{
    Command requested;
    float target;
    {
        ISRGuard guard;
        requested = command;
        if (requested == Command::None)
            return;
        command_active_ = true;
        target = target_deg;
    }
    Result outcome = Result::InvalidCommand;
    if (requested == Command::Move)
        outcome = move(target);
    else
    {
        ISRGuard guard;
        switch (requested)
        {
        case Command::SetZero: outcome = setZero(); break;
        case Command::Enable: outcome = enableOutput(); break;
        case Command::Disable:
            disableOutput(false);
            outcome = Result::Ok;
            break;
        case Command::Stop:
            if (!motor_.isConnected())
                outcome = Result::Offline;
            else if (!enable_requested_ || enable_pending || fault)
                outcome = Result::NotEnabled;
            else
            {
                trajectory_.stop();
                trajectory_.errorUpdate();
                controller_.setRef(0.0f);
                busy = false;
                motion_complete = false;
                target_deg = motor_.getAngle();
                accepted_target_deg = target_deg;
                outcome = Result::Ok;
            }
            break;
        case Command::ApplyLimits: outcome = applyLimits(); break;
        default: break;
        }
    }
    ISRGuard guard;
    result = outcome;
    command = Command::None;
    command_active_ = false;
}

void DMPositionTest::transmitFailed()
{
    disableOutput(true);
    result = Result::TransmitFailed;
}

void DMPositionTest::profileUpdate()
{
    if (!command_active_ && enable_requested_ && !enable_pending && !fault)
    {
        trajectory_.profileUpdate(0.01f);
        if (busy && trajectory_.isFinished() && motor_.isConnected() &&
            std::fabs(motor_.getVelocity()) <= control_.stationary_rpm)
        {
            busy = false;
            motion_complete = true;
        }
    }
}

void DMPositionTest::errorUpdate()
{
    if (!command_active_ && enable_requested_ && !enable_pending && !fault)
        trajectory_.errorUpdate();
}

void DMPositionTest::controllerUpdate()
{
    observeFeedback();
    if (enable_pending)
    {
        if (enable_wait_ticks_ != 0)
            --enable_wait_ticks_;
        else
        {
            disableOutput(true);
            result = Result::MotorFault;
        }
    }
    if (!enable_requested_ || fault)
    {
        motor_.sendDisableFrame();
        commanded_torque_nm = 0.0f;
        return;
    }
    if (limits_configured && !inRange(motor_.getAngle()))
    {
        disableOutput(true);
        result = Result::OutsideLimits;
        return;
    }
    if (enable_pending)
    {
        motor_.setCurrent(0.0f);
        return;
    }
    if (command_active_)
    {
        motor_.setCurrent(commanded_torque_nm);
        return;
    }
    trajectory_.controllerUpdate();
    commanded_torque_nm = controller_.getPID().getOutput();
    if (!std::isfinite(commanded_torque_nm))
    {
        disableOutput(true);
        result = Result::InvalidParameters;
    }
}
} // namespace device
