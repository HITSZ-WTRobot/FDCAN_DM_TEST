#pragma once

#include "dji.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include <cstdint>
#include <optional>

namespace device
{
/** 单台 M2006/C610，使用库的输出轴 deg/rpm 坐标。
 * 在 Can::start() 前构造；借用的总线和本对象必须常驻。
 * 命令/service 由单个任务消费。反馈与所有周期回调必须
 * 共用同一不可相互抢占的 IRQ 优先级。短时 ISRGuard 保护任务交接及
 * 快照；任务侧求解/复位期间保持中断开启，command_active_ 用于阻止
 * 轨迹访问。所有方法均不提交 CAN 帧，也不喂看门狗。
 */
class M2006Position final
{
public:
    struct Config
    {
        PIDMotor::Config velocity_pid;
        PD::Config position_pd;
        trajectory::MotorTrajectory<1>::ProfileConfig profile;
        float finish_deg{0.5f};
        float stationary_rpm{0.1f};
    };

    enum class Result : std::uint32_t
    {
        Ok, Offline, NotZeroed, NotEnabled, Busy, OutsideLimits, NotStationary,
        PlanningFailed, Fault, InvalidParameters, TransmitFailed
    };

    struct Feedback
    {
        Result result{Result::Ok};
        bool connected{false};
        bool zeroed{false};
        bool enabled{false}; // 软件控制器状态，并非 C610 使能确认。
        bool busy{false};
        bool motion_complete{false};
        bool fault{false};
        float position_deg{0.0f};
        float velocity_rpm{0.0f};
        float target_deg{0.0f};
        // 最近实际使用的速度 PID 参考值：轨迹速度前馈 + 位置 PD。
        float velocity_reference_rpm{0.0f};
        std::uint32_t feedback_count{0};
    };

    M2006Position(const motors::DJIMotor::Config& motor_config, const Config& config);
    M2006Position(const M2006Position&) = delete;
    M2006Position& operator=(const M2006Position&) = delete;
    M2006Position(M2006Position&&) = delete;
    M2006Position& operator=(M2006Position&&) = delete;

    // 仅限任务。stop 保持位置；disable 释放控制并缓存零电流。
    Result enable();
    Result disable();
    Result stop();
    Result moveToDegrees(float target_deg); // 有限数值，含端点的 [0, 360] 区间，绝不钳位。
    // 须失能且获得 50 个新的稳定样本；成功时显式清除锁存故障。
    Result setSoftwareZero();
    void service(); // 首次上电自动设零及延后复位故障历史；绝不使能。

    void profileUpdate();             // 100 Hz：实际调用 profileUpdate(0.01f)。
    void positionUpdate();            // 500 Hz：观察新样本并实际调用 errorUpdate。
    void velocityUpdate();            // 1 kHz：实际调用 controllerUpdate，仅缓存输出。
    void transmitFailed();            // 锁存故障，不立即提交 CAN 帧。
    void linkLost(); // 可在 ISR 中安全锁存故障；将缓存置零，不立即输出 CAN 帧。
    void linkRestored() noexcept { host_blocked_=false; }
    Config controlParameters() const noexcept { return config_; }
    [[nodiscard]] Feedback feedback() const;

private:
    using Trajectory = trajectory::MotorTrajectory<1>;
    void feedbackReceived(const bsp::can::FrameView& frame) noexcept;
    static motors::DJIMotor::Config manualZeroConfig(const motors::DJIMotor::Config& config);
    [[nodiscard]] bool validParameters(const motors::DJIMotor::Config& config) const;
    [[nodiscard]] bool inBounds(float position) const;
    [[nodiscard]] bool stableNow() const;
    void resetStability();
    void observeConnection(); // 调用方须处于 IRQ 上下文或持有 ISRGuard。
    void latchFault(Result reason, bool invalidate_zero = false);
    Result recordResult(Result result);
    // 仅限任务，且须设置 command_active_；构造/SCurve 计算期间不屏蔽中断。
    void rebuildTrajectory();

    motors::DJIMotor motor_;
    controllers::MotorVelController controller_;
    const Config config_;
    const std::uint8_t id_;
    const bool parameters_valid_;
    const float encoder_tolerance_deg_; // 输出端实际编码器的两个 LSB，已计入 36:1 减速比。
    std::optional<Trajectory> trajectory_;
    Feedback state_{};
    volatile bool command_active_{false};
    bool reset_pending_{false};
    bool host_blocked_{false};
    bool auto_zero_pending_{false};
    bool ever_connected_{false};
    bool ever_zeroed_{false};
    std::uint32_t observed_feedback_count_{0};
    std::uint32_t move_feedback_count_{0};
    std::uint32_t fault_generation_{0};
    unsigned stable_samples_{0};
    float stable_min_deg_{0.0f};
    float stable_max_deg_{0.0f};
};
} // 命名空间 device
