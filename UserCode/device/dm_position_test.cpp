#include "device/dm_position_test.hpp"
#include "isr_lock.h"
#include <algorithm>
#include <cmath>

namespace device
{
namespace
{
constexpr float RadToDeg = 180.0f / 3.14159265358979323846f;
constexpr unsigned StableSamples = 50;
constexpr float JointHalfRangeDeg = 180.0f;
bool positive(float value) { return std::isfinite(value) && value > 0.0f; }
} // namespace

DMPositionTest::DMPositionTest(const motors::DMMotor::Config& motor_config,
                               const ControlParameters& control, PositionKind kind) :
    motor_(motor_config), control_(control), kind_(kind),
    // Preserve the existing driver's S3519 contract, confirmed by the user's hardware.
    position_scale_(motor_config.reduction_rate *
                    (motor_config.type == motors::DMMotor::Type::S3519 ? 19.203f : 1.0f)),
    velocity_scale_(motor_config.reduction_rate),
    protocol_max_position_deg_(motor_config.pos_max_rad * RadToDeg),
    protocol_max_velocity_deg_s_(motor_config.vel_max_rad * RadToDeg),
    protocol_max_torque_nm_(motor_config.tor_max),
    position_lsb_deg_(protocol_max_position_deg_ / (32767.0f * position_scale_))
{
}

bool DMPositionTest::validControl() const noexcept
{
    return positive(control_.kp) && control_.kp <= 500.0f &&
           positive(control_.kd) && control_.kd <= 5.0f &&
           std::isfinite(control_.torque_ff_nm) &&
           std::fabs(control_.torque_ff_nm) <= protocol_max_torque_nm_ &&
           positive(control_.finish_deg) && positive(control_.stationary_rpm) &&
           positive(position_scale_) && positive(velocity_scale_);
}

bool DMPositionTest::validReference(float position, float velocity) const noexcept
{
    return validAbsoluteReference(zero_output_deg_ + position, velocity);
}

bool DMPositionTest::validAbsoluteReference(float position, float velocity) const noexcept
{
    const float wire_position = position * position_scale_;
    const float wire_velocity = velocity * velocity_scale_;
    return std::isfinite(wire_position) && std::isfinite(wire_velocity) &&
           std::fabs(wire_position) <= protocol_max_position_deg_ &&
           std::fabs(wire_velocity) <= protocol_max_velocity_deg_s_;
}

bool DMPositionTest::validLimits(const MotionLimits& l, float zero) const noexcept
{
    const bool valid_dynamics = positive(l.max_velocity_deg_s) &&
        l.max_velocity_deg_s * velocity_scale_ <= protocol_max_velocity_deg_s_ &&
        positive(l.max_acceleration_deg_s2) && positive(l.max_jerk_deg_s3);
    if (kind_ == PositionKind::LinearActuator)
        return valid_dynamics; // No software travel interval; encoding is checked on every reference.
    return valid_dynamics && std::isfinite(l.min_deg) && std::isfinite(l.max_deg) &&
           l.min_deg < l.max_deg && l.min_deg >= -JointHalfRangeDeg && l.max_deg <= JointHalfRangeDeg &&
           validAbsoluteReference(zero + l.min_deg, 0.0f) &&
           validAbsoluteReference(zero + l.max_deg, 0.0f);
}

bool DMPositionTest::inRange(float position) const noexcept
{
    return std::isfinite(position) && (kind_ == PositionKind::LinearActuator ||
           (position >= limits_.min_deg && position <= limits_.max_deg));
}

float DMPositionTest::currentPosition() const noexcept
{
    return motor_.getAngle() - zero_output_deg_;
}

void DMPositionTest::feedbackReceived() noexcept
{
    feedback_count = feedback_count + 1;
}

void DMPositionTest::holdPosition(float position) noexcept
{
    profile_.reset();
    elapsed_s_ = duration_s_ = 0.0f;
    reference_deg = position;
    reference_velocity_deg_s = 0.0f;
    target_deg = position;
    if (kind_ == PositionKind::LinearActuator)
        target_mm = position * LinearMmPerDegree;
    accepted_target_deg = position;
    busy = false;
    motion_complete = false;
}

void DMPositionTest::disableOutput(bool latch_fault)
{
    holdPosition(currentPosition());
    commanded_torque_ff_nm = 0.0f;
    enable_requested_ = false;
    enable_pending = false;
    enable_phase_ = EnablePhase::None;
    enable_wait_ticks_ = 0;
    if (latch_fault)
        fault = true;
    // Do not declare physical disable: fresh feedback must confirm it.
    // State change only. CAN output belongs exclusively to this motor's next TX slot.
}

void DMPositionTest::feedbackUpdate()
{
    connected = motor_.isConnected();
    if (!connected)
    {
        stable_samples_ = 0;
        enabled = false; // No current confirmation, not proof of physical disable.
        if (ever_connected_ && !fault)
        {
            disableOutput(true);
            result = Result::Offline;
        }
        return;
    }
    ever_connected_ = true;
    if (enable_pending)
    {
        if (enable_wait_ticks_ == 0)
        {
            disableOutput(true);
            result = Result::MotorFault;
            return;
        }
        --enable_wait_ticks_;
    }
    const auto count = feedback_count;
    if (count == observed_feedback_count_)
        return; // Cached feedback does not advance zeroing or confirm arrival.
    observed_feedback_count_ = count;
    position_deg = currentPosition();
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
    if (enable_requested_)
    {
        if (enable_pending && enable_phase_ == EnablePhase::WaitEnable && enabled)
        {
            enable_pending = false;
            enable_phase_ = EnablePhase::None;
        }
        else if (enable_pending && enable_phase_ == EnablePhase::WaitPreload &&
                 count != preload_feedback_count_ && motor_state == 0)
        {
            enable_phase_ = EnablePhase::SendEnable;
            enable_wait_ticks_ = WATCHDOG_SNACKS;
        }
        else if ((!enable_pending && !enabled) ||
                 (enable_pending && enabled && enable_phase_ != EnablePhase::WaitEnable))
        {
            disableOutput(true);
            result = Result::MotorFault;
            return;
        }
    }
    if (enable_requested_ && !enable_pending && !fault)
    {
        if (busy && elapsed_s_ >= duration_s_ &&
            std::fabs(position_deg - accepted_target_deg) <= control_.finish_deg &&
            std::fabs(velocity_rpm) <= control_.stationary_rpm)
        {
            busy = false;
            motion_complete = true;
        }
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
        if (high - low > 2.0f * position_lsb_deg_)
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
        result = setZero();
}

Result DMPositionTest::applyLimits()
{
    if (enable_requested_ || motor_.state() != motors::DMMotor::State::Disabled)
        return Result::Busy;
    if (!zeroed)
        return Result::NotZeroed;
    const MotionLimits staged{min_deg, max_deg, max_velocity_deg_s,
                              max_acceleration_deg_s2, max_jerk_deg_s3};
    if (!validLimits(staged, zero_output_deg_))
        return Result::InvalidParameters;
    limits_ = staged;
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
    const float new_zero = motor_.getAngle();
    MotionLimits candidate = limits_configured ? limits_ : MotionLimits{
        min_deg, max_deg, max_velocity_deg_s, max_acceleration_deg_s2, max_jerk_deg_s3};
    if (kind_ == PositionKind::Joint)
    {
        candidate.min_deg = -JointHalfRangeDeg;
        candidate.max_deg = JointHalfRangeDeg;
    }
    // Validate before changing the origin: failure leaves the old zero, target and limits intact.
    if (!validAbsoluteReference(new_zero, 0.0f) || !validLimits(candidate, new_zero))
        return Result::InvalidParameters;
    zero_output_deg_ = new_zero;
    limits_ = candidate;
    limits_configured = true;
    min_deg = candidate.min_deg;
    max_deg = candidate.max_deg;
    holdPosition(0.0f);
    position_deg = 0.0f;
    zeroed = true;
    stable_samples_ = 0;
    return Result::Ok;
}

Result DMPositionTest::enableOutput()
{
    if (host_link_blocked_)
        return Result::NotEnabled;
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
    const float current = currentPosition();
    if (!inRange(current) || !validReference(current, 0.0f))
        return Result::OutsideLimits;
    holdPosition(current);
    enable_requested_ = true;
    enable_pending = true;
    enable_phase_ = EnablePhase::Preload;
    enable_wait_ticks_ = WATCHDOG_SNACKS;
    fault = false;
    return Result::Ok; // Accepted; TIM6 preloads and requests enable, RX confirms it.
}

Result DMPositionTest::move(float target)
{
    float start;
    velocity_profile::SCurveProfile::Config config{};
    {
        ISRGuard guard;
        if (!motor_.isConnected())
            return Result::Offline;
        if (!enable_requested_ || enable_pending || fault || !enabled)
            return Result::NotEnabled;
        if (busy)
            return Result::Busy;
        if (!inRange(target) || !inRange(currentPosition()) || !validReference(target, 0.0f))
            return Result::OutsideLimits;
        if (std::fabs(motor_.getVelocity()) > control_.stationary_rpm ||
            std::fabs(currentPosition() - reference_deg) > control_.finish_deg)
            return Result::NotStationary;
        start = reference_deg;
        config = {limits_.max_velocity_deg_s, limits_.max_acceleration_deg_s2,
                  limits_.max_jerk_deg_s3};
    }
    // One existing solver, no external PID; solving does not mask interrupts.
    const velocity_profile::SCurveProfile planned(config, start, 0.0f, 0.0f, target);
    ISRGuard guard;
    if (fault || !enable_requested_ || !motor_.isConnected())
        return Result::Offline;
    if (!planned.success() || !std::isfinite(planned.getTotalTime()) || planned.getTotalTime() < 0)
        return Result::PlanningFailed;
    profile_ = planned;
    elapsed_s_ = 0.0f;
    duration_s_ = planned.getTotalTime();
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
        target = requested == Command::MoveMm ? target_mm / LinearMmPerDegree : target_deg;
    }
    Result outcome = Result::InvalidCommand;
    if (requested == Command::Move || requested == Command::MoveMm)
    {
        const bool matching_unit = (requested == Command::MoveMm) ==
                                   (kind_ == PositionKind::LinearActuator);
        if (matching_unit)
            outcome = move(target);
    }
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
                holdPosition(currentPosition());
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

Result DMPositionTest::execute(Command requested, float target, const MotionLimits* limits)
{
    {
        ISRGuard guard;
        if (command_active_ || command != Command::None || (requested == Command::MoveMm && busy))
            return Result::Busy;
        if (requested == Command::Move)
            target_deg = target;
        if (requested == Command::MoveMm)
            target_mm = target;
        if (requested == Command::ApplyLimits && limits)
        {
            min_deg = limits->min_deg;
            max_deg = limits->max_deg;
            max_velocity_deg_s = limits->max_velocity_deg_s;
            max_acceleration_deg_s2 = limits->max_acceleration_deg_s2;
            max_jerk_deg_s3 = limits->max_jerk_deg_s3;
        }
        command = requested;
    }
    commandUpdate();
    ISRGuard guard;
    return result;
}

void DMPositionTest::linkLost()
{
    disableOutput(true);
    command = Command::None;
    result = Result::Offline;
    host_link_blocked_ = true;
}

Result DMPositionTest::moveToMm(float mm)
{
    if (kind_ != PositionKind::LinearActuator)
        return Result::InvalidCommand;
    if (!std::isfinite(mm) || !std::isfinite(mm / LinearMmPerDegree))
        return Result::OutsideLimits;
    return execute(Command::MoveMm, mm);
}

Result DMPositionTest::setSoftwareZero()
{
    return execute(Command::SetZero);
}

std::optional<LinearFeedback> DMPositionTest::linearFeedback() const
{
    if (kind_ != PositionKind::LinearActuator)
        return std::nullopt;
    ISRGuard guard;
    return LinearFeedback{position_deg * LinearMmPerDegree,
                          velocity_rpm * (LinearMmPerRevolution / 60.0f),
                          accepted_target_deg * LinearMmPerDegree,
                          reference_deg * LinearMmPerDegree,
                          reference_velocity_deg_s * LinearMmPerDegree};
}

void DMPositionTest::profileUpdate()
{
    if (command_active_ || !enable_requested_ || enable_pending || fault || !busy || !profile_)
        return;
    elapsed_s_ = std::min(elapsed_s_ + 0.01f, duration_s_);
    const float position = elapsed_s_ >= duration_s_ ? accepted_target_deg : profile_->CalcX(elapsed_s_);
    const float velocity = elapsed_s_ >= duration_s_ ? 0.0f : profile_->CalcV(elapsed_s_);
    if (!inRange(position) || !validReference(position, velocity))
    {
        disableOutput(true);
        result = Result::OutsideLimits;
        return;
    }
    reference_deg = position;
    reference_velocity_deg_s = velocity;
}

void DMPositionTest::sendReference(float torque_ff)
{
    const float position = reference_deg;
    const float velocity = reference_velocity_deg_s;
    if (!validReference(position, velocity))
    {
        disableOutput(true);
        result = Result::OutsideLimits;
        motor_.disable(); // Already inside the TX slot: submit exactly one disable frame.
        return;
    }
    // setInternalMIT applies direction and deg->rad. It does not add the software
    // origin or S3519's position scale, so perform only that coordinate adaptation.
    motor_.setInternalMIT(torque_ff, (zero_output_deg_ + position) * position_scale_,
                          velocity * velocity_scale_, control_.kp, control_.kd);
    commanded_torque_ff_nm = torque_ff; // Feedforward only, not total motor torque.
}

void DMPositionTest::sendUpdate()
{
    if (!enable_requested_ || fault)
    {
        motor_.disable();
        commanded_torque_ff_nm = 0.0f;
        return;
    }
    if (!motor_.isConnected())
    {
        disableOutput(true);
        result = Result::Offline;
        motor_.disable();
        return;
    }
    if (!inRange(currentPosition()))
    {
        disableOutput(true);
        result = Result::OutsideLimits;
        motor_.disable();
        return;
    }
    if (enable_phase_ == EnablePhase::Preload)
    {
        preload_feedback_count_ = feedback_count;
        sendReference(0.0f);
        if (!fault)
            enable_phase_ = EnablePhase::WaitPreload;
    }
    else if (enable_phase_ == EnablePhase::SendEnable)
    {
        (void)motor_.enable();
        enable_phase_ = EnablePhase::WaitEnable;
        enable_wait_ticks_ = WATCHDOG_SNACKS;
    }
    else
        sendReference(enable_pending ? 0.0f : control_.torque_ff_nm);
}

void DMPositionTest::transmitFailed()
{
    disableOutput(true);
    result = Result::TransmitFailed;
}
} // namespace device
