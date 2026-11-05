#include "m2006_position.hpp"
#include "isr_lock.h"

#include <algorithm>
#include <cmath>

namespace device
{
namespace
{
constexpr unsigned kStableSamples = 50;

bool positiveFinite(float value)
{
    return std::isfinite(value) && value > 0.0f;
}

bool nonnegativeFinite(float value)
{
    return std::isfinite(value) && value >= 0.0f;
}
} // 匿名命名空间

motors::DJIMotor::Config M2006Position::manualZeroConfig(const motors::DJIMotor::Config& config)
{
    auto result = config;
    // 底层驱动的 50 帧自动设零并不检查样本是否静止、稳定。
    result.auto_zero = false;
    return result;
}

M2006Position::M2006Position(const motors::DJIMotor::Config& motor_config, const Config& config) :
    motor_(manualZeroConfig(motor_config)),
    controller_(&motor_, {config.velocity_pid, controllers::ControlMode::ExternalPID}),
    config_(config), id_(motor_config.id1), parameters_valid_(validParameters(motor_config)),
    encoder_tolerance_deg_(positiveFinite(motor_config.reduction_rate)
                                   ? 2.0f * 360.0f / 8192.0f / 36.0f / motor_config.reduction_rate
                                   : 0.0f)
{
    // 不使能、不启动硬件、不发送报文。配置无效时不执行控制动作。
    // DJIMotor 已在成员构造期间、注册本观察回调之前注册其解码器。
    if (motor_config.can.register_callback<&M2006Position::feedbackReceived>(*this) != bsp::can::Status::Ok)
        Error_Handler();
    if (parameters_valid_)
        trajectory_.emplace(&controller_, config_.profile, config_.position_pd, config_.finish_deg);
    else
        state_.result = Result::InvalidParameters;
}

bool M2006Position::validParameters(const motors::DJIMotor::Config& config) const
{
    const auto& pid = config_.velocity_pid;
    const auto& pd = config_.position_pd;
    return config.type == motors::DJIMotor::Type::M2006_C610 &&
           positiveFinite(config.reduction_rate) &&
           nonnegativeFinite(pid.Kp) && nonnegativeFinite(pid.Ki) && nonnegativeFinite(pid.Kd) &&
           positiveFinite(pid.abs_output_max) && pid.abs_output_max <= 10000.0f &&
           nonnegativeFinite(pd.Kp) && nonnegativeFinite(pd.Kd) &&
           positiveFinite(pd.abs_output_max) && positiveFinite(config_.profile.max_spd) &&
           positiveFinite(config_.profile.max_acc) && positiveFinite(config_.profile.max_jerk) &&
           positiveFinite(config_.finish_deg) && positiveFinite(config_.stationary_rpm);
}

bool M2006Position::inBounds(float position) const
{
    return std::isfinite(position) && position >= -encoder_tolerance_deg_ &&
           position <= 360.0f + encoder_tolerance_deg_;
}

bool M2006Position::stableNow() const
{
    // 包含最新原始帧，即使该帧在 500 Hz 观察之后才到达。
    const float position = motor_.getAngle();
    const float velocity = motor_.getVelocity();
    return stable_samples_ >= kStableSamples && motor_.isConnected() &&
           std::isfinite(position) && std::isfinite(velocity) &&
           std::fabs(velocity) <= config_.stationary_rpm &&
           std::max(stable_max_deg_, position) - std::min(stable_min_deg_, position) <=
                   encoder_tolerance_deg_;
}

void M2006Position::resetStability()
{
    stable_samples_ = 0;
    auto_zero_pending_ = false;
}

M2006Position::Result M2006Position::recordResult(Result result)
{
    // 被拒绝的命令不能清除遥测中锁存故障的原因。
    if (!state_.fault)
        state_.result = result;
    return result;
}

void M2006Position::latchFault(Result reason, bool invalidate_zero)
{
    ++fault_generation_;
    if (!state_.fault)
        state_.result = reason;
    state_.fault = true;
    state_.enabled = false;
    state_.busy = false;
    state_.motion_complete = false;
    state_.velocity_reference_rpm = 0.0f;
    if (invalidate_zero)
        state_.zeroed = false;
    resetStability();
    // 此处绝不访问 trajectory_：任务可能正在求解或重建它。
    // 释放控制也会让驱动在下一个聚合发送时隙明确输出零电流。
    controller_.disable();
    motor_.setCurrent(0.0f);
    reset_pending_ = true;
}

void M2006Position::observeConnection()
{
    const bool was_connected = state_.connected;
    state_.connected = state_.feedback_count != 0 && motor_.isConnected();
    if (!state_.connected)
    {
        resetStability();
        if (ever_connected_ && (was_connected || !state_.fault || state_.zeroed))
            latchFault(Result::Offline, true); // 无法获知通信中断期间编码器的转数。
        return;
    }
    if (!std::isfinite(state_.position_deg) || !std::isfinite(state_.velocity_rpm))
    {
        if (!state_.fault || state_.zeroed)
            latchFault(Result::Fault, true);
        resetStability();
    }
    else if (!state_.fault && state_.zeroed && !inBounds(state_.position_deg))
        latchFault(Result::OutsideLimits);
    // 不在每个定时周期重复锁存同一越界故障：失能状态下仍须
    // 累积新的静止样本，以便显式重新设置软件零点。
}

void M2006Position::rebuildTrajectory()
{
    {
        ISRGuard guard;
        controller_.disable();
        motor_.setCurrent(0.0f);
        state_.enabled = false;
        state_.velocity_reference_rpm = 0.0f;
    }
    // 重建实际对象，以清除其原本不可直接访问的位置 PD 历史。
    // 该标志仅阻止定时器访问，不阻止反馈/故障观察。SCurve
    // 构造期间不屏蔽 IRQ，即使是初始的零长度曲线也如此。
    trajectory_.reset();
    controller_.getPID().reset();
    controller_.setRef(0.0f);
    if (parameters_valid_)
        trajectory_.emplace(&controller_, config_.profile, config_.position_pd, config_.finish_deg);
}

M2006Position::Result M2006Position::enable()
{
    {
        ISRGuard guard;
        if (host_blocked_)
            return Result::NotEnabled;
        if (command_active_)
            return recordResult(Result::Busy);
        observeConnection();
        if (!parameters_valid_)
            return recordResult(Result::InvalidParameters);
        if (!state_.connected)
            return recordResult(Result::Offline);
        if (state_.fault)
            return state_.result;
        if (!state_.zeroed)
            return recordResult(Result::NotZeroed);
        if (state_.enabled)
            return recordResult(state_.busy ? Result::Busy : Result::Ok);
        command_active_ = true;
        reset_pending_ = false;
    }
    rebuildTrajectory();
    ISRGuard guard;
    observeConnection();
    if (state_.fault || !state_.connected)
    {
        command_active_ = false;
        return state_.fault ? state_.result : recordResult(Result::Offline);
    }
    const bool enabled = trajectory_->enable();
    state_.enabled = enabled;
    state_.busy = false;
    state_.motion_complete = false;
    state_.target_deg = motor_.getAngle();
    resetStability();
    command_active_ = false;
    return recordResult(enabled ? Result::Ok : Result::NotEnabled);
}

M2006Position::Result M2006Position::disable()
{
    {
        ISRGuard guard;
        if (command_active_)
            return recordResult(Result::Busy);
        command_active_ = true;
        state_.busy = false;
        state_.motion_complete = false;
        reset_pending_ = false;
        resetStability();
    }
    rebuildTrajectory();
    ISRGuard guard;
    observeConnection();
    command_active_ = false;
    return state_.fault ? state_.result : recordResult(Result::Ok);
}

M2006Position::Result M2006Position::stop()
{
    ISRGuard guard;
    if (command_active_)
        return recordResult(Result::Busy);
    observeConnection();
    if (!state_.connected)
        return recordResult(Result::Offline);
    if (state_.fault)
        return state_.result;
    if (!state_.enabled)
        return recordResult(Result::NotEnabled);
    trajectory_->stop();
    controller_.setRef(0.0f);
    state_.target_deg = motor_.getAngle();
    state_.busy = false;
    state_.motion_complete = false; // 取消运动绝不报告为到位。
    return recordResult(Result::Ok);
}

M2006Position::Result M2006Position::moveToDegrees(float target_deg)
{
    {
        ISRGuard guard;
        if (host_blocked_)
            return Result::NotEnabled;
        if (command_active_)
            return recordResult(Result::Busy);
        observeConnection();
        if (!parameters_valid_ || !std::isfinite(target_deg))
            return recordResult(Result::InvalidParameters);
        if (target_deg < 0.0f || target_deg > 360.0f)
            return recordResult(Result::OutsideLimits);
        if (!state_.connected)
            return recordResult(Result::Offline);
        if (state_.fault)
            return state_.result;
        if (!state_.zeroed)
            return recordResult(Result::NotZeroed);
        if (!state_.enabled)
            return recordResult(Result::NotEnabled);
        if (state_.busy)
            return recordResult(Result::Busy);
        command_active_ = true;
    }
    // 已停止的轨迹从实测位置起步，规划初速度为零。
    // 否则由上一条已完成曲线提供一致的位置/速度对；
    // 不混用原始反馈读数、不复制求解器，也不提前停止旧曲线。
    const bool planned = trajectory_->setTarget(target_deg, trajectory::LinkMode::PreviousCurve);
    ISRGuard guard;
    observeConnection();
    if (state_.fault || !state_.connected)
    {
        command_active_ = false;
        return state_.fault ? state_.result : recordResult(Result::Offline);
    }
    if (planned)
    {
        state_.target_deg = target_deg;
        state_.busy = true;
        state_.motion_complete = false;
        move_feedback_count_ = state_.feedback_count;
    }
    // 库在失败时保留上一轨迹；此处也保留目标及到位状态。
    command_active_ = false;
    return recordResult(planned ? Result::Ok : Result::PlanningFailed);
}

M2006Position::Result M2006Position::setSoftwareZero()
{
    std::uint32_t generation;
    {
        ISRGuard guard;
        if (command_active_)
            return recordResult(Result::Busy);
        observeConnection();
        if (!parameters_valid_)
            return recordResult(Result::InvalidParameters);
        if (!state_.connected)
            return recordResult(Result::Offline);
        if (state_.enabled)
            return recordResult(Result::Busy);
        if (!stableNow())
            return recordResult(Result::NotStationary);
        command_active_ = true;
        generation = fault_generation_;
        reset_pending_ = false;
    }
    rebuildTrajectory();
    ISRGuard guard;
    observeConnection();
    if (generation != fault_generation_ || !state_.connected || !stableNow())
    {
        command_active_ = false;
        if (!state_.connected)
            return recordResult(Result::Offline);
        return state_.fault ? state_.result : recordResult(Result::NotStationary);
    }
    motor_.resetAngle(); // 受保护，不会与底层驱动的编码器/转数更新冲突。
    state_.position_deg = 0.0f;
    state_.target_deg = 0.0f;
    state_.busy = false;
    state_.motion_complete = false;
    state_.zeroed = true;
    state_.fault = false;
    state_.result = Result::Ok;
    ever_zeroed_ = true;
    resetStability();
    command_active_ = false;
    return Result::Ok;
}

void M2006Position::service()
{
    bool reset;
    bool auto_zero;
    {
        ISRGuard guard;
        if (command_active_)
            return;
        observeConnection();
        reset = reset_pending_;
        auto_zero = auto_zero_pending_ && !ever_zeroed_ && !state_.fault;
        if (reset)
        {
            command_active_ = true;
            reset_pending_ = false;
        }
    }
    if (reset)
    {
        rebuildTrajectory();
        ISRGuard guard;
        command_active_ = false;
    }
    if (auto_zero)
        setSoftwareZero(); // 在保护范围内重新检查最新连接、速度和稳定性。
}

void M2006Position::feedbackReceived(const bsp::can::FrameView& frame) noexcept
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
}

void M2006Position::profileUpdate()
{
    observeConnection();
    if (!command_active_ && state_.enabled && !state_.fault)
        trajectory_->profileUpdate(0.01f);
}

void M2006Position::positionUpdate()
{
    observeConnection();
    if (!state_.connected)
        return;
    const bool fresh = observed_feedback_count_ != state_.feedback_count;
    if (fresh)
    {
        observed_feedback_count_ = state_.feedback_count;
        if (!state_.enabled && parameters_valid_ && std::isfinite(state_.position_deg) &&
            std::isfinite(state_.velocity_rpm) &&
            std::fabs(state_.velocity_rpm) <= config_.stationary_rpm)
        {
            const float minimum = std::min(stable_min_deg_, state_.position_deg);
            const float maximum = std::max(stable_max_deg_, state_.position_deg);
            if (stable_samples_ == 0 || maximum - minimum > encoder_tolerance_deg_)
            {
                stable_min_deg_ = stable_max_deg_ = state_.position_deg;
                stable_samples_ = 1;
            }
            else
            {
                stable_min_deg_ = minimum;
                stable_max_deg_ = maximum;
                if (stable_samples_ < kStableSamples)
                    ++stable_samples_;
            }
            auto_zero_pending_ = stable_samples_ >= kStableSamples && !ever_zeroed_ && !state_.fault;
        }
        else
            resetStability();
    }
    if (command_active_ || !state_.enabled || state_.fault)
        return;
    trajectory_->errorUpdate();
    if (fresh && state_.busy && state_.feedback_count != move_feedback_count_ &&
        trajectory_->isFinished() &&
        std::fabs(state_.position_deg - state_.target_deg) <= config_.finish_deg &&
        std::fabs(state_.velocity_rpm) <= config_.stationary_rpm)
    {
        state_.busy = false;
        state_.motion_complete = true;
    }
}

void M2006Position::velocityUpdate()
{
    observeConnection();
    if (command_active_ || !state_.enabled || state_.fault)
        return;
    trajectory_->controllerUpdate();
    state_.velocity_reference_rpm = controller_.getPID().getRef();
}

void M2006Position::transmitFailed()
{
    ISRGuard guard;
    latchFault(Result::TransmitFailed);
}

void M2006Position::linkLost()
{
    ISRGuard guard;
    host_blocked_=true;
    latchFault(Result::Offline);
}

M2006Position::Feedback M2006Position::feedback() const
{
    ISRGuard guard;
    return state_;
}
} // 命名空间 device
