#include "m2006_velocity.hpp"
#include "isr_lock.h"

#include <cmath>

namespace device
{
namespace
{
constexpr unsigned kStationarySamples = 50;

bool positiveFinite(float value)
{
    return std::isfinite(value) && value > 0.0f;
}

bool nonnegativeFinite(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}
} // 匿名命名空间

motors::DJIMotor::Config M2006Velocity::diagnosticAngleConfig(const motors::DJIMotor::Config& config)
{
    auto result = config;
    result.auto_zero = false;
    return result;
}

M2006Velocity::M2006Velocity(const motors::DJIMotor::Config& motor_config, const Config& config) :
    motor_(diagnosticAngleConfig(motor_config)),
    controller_(&motor_, {config.velocity_pid, controllers::ControlMode::ExternalPID}),
    config_(config), id_(motor_config.id1),
    motor_parameters_valid_(motor_config.type == motors::DJIMotor::Type::M2006_C610 &&
                            positiveFinite(motor_config.reduction_rate))
{
    // 驱动解码器先注册，观察回调随后读取真实的输出轴反馈。
    if (motor_config.can.register_callback<&M2006Velocity::feedbackReceived>(*this) != bsp::can::Status::Ok)
        Error_Handler();
    if (!validParameters())
        state_.result = Result::InvalidParameters;
}

bool M2006Velocity::validParameters()
{
    // Ozone 可修改实际 PID 和 config_；执行时校验，不缓存启动时的判断。
    const auto& pid = controller_.getPID().getConfig();
    return motor_parameters_valid_ &&
           nonnegativeFinite(pid.Kp) && nonnegativeFinite(pid.Ki) && nonnegativeFinite(pid.Kd) &&
           positiveFinite(pid.abs_output_max) && pid.abs_output_max <= 10000.0f &&
           positiveFinite(config_.max_rpm) && positiveFinite(config_.stationary_rpm);
}

bool M2006Velocity::stationaryNow() const
{
    return stationary_samples_ >= kStationarySamples && motor_.isConnected() &&
           std::isfinite(motor_.getVelocity()) &&
           std::fabs(motor_.getVelocity()) <= config_.stationary_rpm;
}

void M2006Velocity::resetStability()
{
    stationary_samples_ = 0;
}

void M2006Velocity::resetControl()
{
    controller_.setRef(0.0f);
    controller_.getPID().reset();
    motor_.setCurrent(0.0f);
    state_.target_rpm = 0.0f;
    state_.velocity_reference_rpm = 0.0f;
}

M2006Velocity::Result M2006Velocity::recordResult(Result result)
{
    // 拒绝命令不覆盖锁存故障的最初原因。
    if (!state_.fault)
        state_.result = result;
    return result;
}

void M2006Velocity::latchFault(Result reason)
{
    if (!state_.fault)
        state_.result = reason;
    state_.fault = true;
    state_.enabled = false;
    controller_.disable();
    resetControl();
    resetStability();
}

void M2006Velocity::observeConnection()
{
    const bool was_connected = state_.connected;
    state_.connected = state_.feedback_count != 0 && motor_.isConnected();
    if (!state_.connected)
    {
        resetStability();
        if (ever_connected_ && (was_connected || !state_.fault))
            latchFault(Result::Offline);
        return;
    }
    if (!validParameters())
    {
        latchFault(Result::InvalidParameters);
        return;
    }
    if (!std::isfinite(state_.velocity_rpm))
        latchFault(Result::Fault);
    else if (state_.enabled && std::fabs(state_.target_rpm) > config_.max_rpm)
        latchFault(Result::OutsideLimits); // 在线降低限速不能保留超限参考或静默夹值。
    // 角度仅诊断，任意正反向累计圈数均不触发位置边界故障。
}

M2006Velocity::Result M2006Velocity::enable()
{
    ISRGuard guard;
    observeConnection();
    if (host_blocked_)
        return recordResult(Result::LinkFault);
    if (!validParameters())
        return recordResult(Result::InvalidParameters);
    if (!state_.connected)
        return recordResult(Result::Offline);
    if (state_.enabled)
        return recordResult(Result::Busy);
    if (!stationaryNow())
        return recordResult(Result::NotStationary);
    resetControl();
    state_.enabled = controller_.enable();
    resetStability();
    if (!state_.enabled)
        return recordResult(Result::NotEnabled);
    // 仅此显式命令可恢复故障；旧速度目标绝不重新授权。
    state_.fault = false;
    state_.result = Result::Ok;
    return Result::Ok;
}

M2006Velocity::Result M2006Velocity::disable()
{
    ISRGuard guard;
    controller_.disable();
    state_.enabled = false;
    resetControl();
    resetStability();
    observeConnection();
    return state_.fault ? state_.result : recordResult(Result::Ok);
}

M2006Velocity::Result M2006Velocity::stop()
{
    ISRGuard guard;
    observeConnection();
    if (!state_.connected)
        return recordResult(Result::Offline);
    if (state_.fault)
        return state_.result;
    if (!state_.enabled)
        return recordResult(Result::NotEnabled);
    resetControl();
    return recordResult(Result::Ok);
}

M2006Velocity::Result M2006Velocity::setVelocityRpm(float target_rpm)
{
    ISRGuard guard;
    observeConnection();
    if (host_blocked_)
        return recordResult(Result::LinkFault);
    if (!validParameters() || !std::isfinite(target_rpm))
        return recordResult(Result::InvalidParameters);
    if (std::fabs(target_rpm) > config_.max_rpm)
        return recordResult(Result::OutsideLimits);
    if (!state_.connected)
        return recordResult(Result::Offline);
    if (state_.fault)
        return state_.result;
    if (!state_.enabled)
        return recordResult(Result::NotEnabled);
    controller_.setRef(target_rpm);
    state_.target_rpm = target_rpm;
    return recordResult(Result::Ok);
}

void M2006Velocity::service()
{
    ISRGuard guard;
    observeConnection();
}

void M2006Velocity::feedbackReceived(const bsp::can::FrameView& frame) noexcept
{
    if (frame.header.id_type != bsp::can::IdType::Standard ||
        frame.header.format != bsp::can::FrameFormat::Classic ||
        frame.header.type != bsp::can::FrameType::Data || frame.data.size() != 8 ||
        frame.header.id != 0x200U + id_)
        return;
    ++state_.feedback_count;
    state_.position_deg = motor_.getAngle();
    state_.velocity_rpm = motor_.getVelocity();
    state_.connected = motor_.isConnected();
    ever_connected_ = true;
    // 每个原始新帧恰计一次；500 Hz/1 kHz 回调不能重复累计静止样本。
    if (!state_.enabled && state_.connected && std::isfinite(state_.velocity_rpm) &&
        std::fabs(state_.velocity_rpm) <= config_.stationary_rpm)
    {
        if (stationary_samples_ < kStationarySamples)
            ++stationary_samples_;
    }
    else
        resetStability();
}

void M2006Velocity::feedbackUpdate()
{
    observeConnection();
}

void M2006Velocity::velocityUpdate()
{
    observeConnection();
    if (!state_.enabled || state_.fault)
        return;
    controller_.update();
    state_.velocity_reference_rpm = controller_.getPID().getRef();
}

void M2006Velocity::transmitFailed()
{
    ISRGuard guard;
    latchFault(Result::TransmitFailed);
}

void M2006Velocity::linkLost()
{
    ISRGuard guard;
    host_blocked_ = true;
    latchFault(Result::LinkFault);
}

void M2006Velocity::linkRestored()
{
    ISRGuard guard;
    host_blocked_ = false;
}

M2006Velocity::Config M2006Velocity::controlParameters() const
{
    ISRGuard guard;
    return config_;
}

M2006Velocity::Feedback M2006Velocity::feedback() const
{
    ISRGuard guard;
    return state_;
}
} // 命名空间 device
