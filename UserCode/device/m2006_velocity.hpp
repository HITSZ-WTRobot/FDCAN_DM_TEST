#pragma once

#include "dji.hpp"
#include "motor_vel_controller.hpp"
#include <cstdint>

namespace device
{
/** 单台 M2006/C610，输出轴速度单位 rpm，角度仅作诊断。
 * 在 Can::start() 前构造；借用的总线和本对象必须常驻。
 * 命令/service 由单个任务消费。反馈与周期回调共用同一不可相互抢占的
 * IRQ 优先级；任务侧仅用短时 ISRGuard 交接。所有方法均不提交 CAN 帧，
 * 也不喂看门狗，电流仅由 TIM6 聚合发送。
 */
class M2006Velocity final
{
public:
    struct Config
    {
        PIDMotor::Config velocity_pid;
        float max_rpm{300.0f};
        float stationary_rpm{0.1f};
    };

    enum class Result : std::uint32_t
    {
        Ok, Offline, NotEnabled, Busy, OutsideLimits, NotStationary,
        Fault, InvalidParameters, TransmitFailed, LinkFault
    };

    struct Feedback
    {
        Result result{Result::Ok};
        bool connected{false};
        bool enabled{false}; // 软件控制器状态，并非 C610 使能确认。
        bool fault{false};
        float position_deg{0.0f}; // 连续角度，仅诊断；不重设零点或限制圈数。
        float velocity_rpm{0.0f};
        float target_rpm{0.0f};
        float velocity_reference_rpm{0.0f};
        std::uint32_t feedback_count{0};
    };

    M2006Velocity(const motors::DJIMotor::Config& motor_config, const Config& config);
    M2006Velocity(const M2006Velocity&) = delete;
    M2006Velocity& operator=(const M2006Velocity&) = delete;
    M2006Velocity(M2006Velocity&&) = delete;
    M2006Velocity& operator=(M2006Velocity&&) = delete;

    // 仅限任务。Enable（含故障恢复）须失能后收到 50 个新的低速反馈。
    Result enable();
    Result disable();
    Result stop(); // 清 PID 并授权 0 rpm，保持软件使能，不锁位置。
    Result setVelocityRpm(float target_rpm); // 有限且 |rpm| <= max_rpm；拒绝时保留旧参考。
    void service(); // 仅观察故障；绝不自动恢复或使能。
    void feedbackUpdate(); // 500 Hz：观察连接/参数/反馈故障。
    void velocityUpdate(); // 1 kHz：库速度 PID，仅缓存电流。
    void transmitFailed();
    void linkLost();
    void linkRestored();
    [[nodiscard]] Config controlParameters() const;
    [[nodiscard]] Feedback feedback() const;

private:
    void feedbackReceived(const bsp::can::FrameView& frame) noexcept;
    static motors::DJIMotor::Config diagnosticAngleConfig(const motors::DJIMotor::Config& config);
    [[nodiscard]] bool validParameters();
    [[nodiscard]] bool stationaryNow() const;
    void resetControl(); // 调用方处于 IRQ 上下文或持有 ISRGuard。
    void resetStability();
    void observeConnection();
    void latchFault(Result reason);
    Result recordResult(Result result);

    motors::DJIMotor motor_;
    controllers::MotorVelController controller_;
    Config config_; // 保留真实对象路径，供 Ozone 显式调参。
    const std::uint8_t id_;
    const bool motor_parameters_valid_;
    Feedback state_{};
    bool host_blocked_{false};
    bool ever_connected_{false};
    unsigned stationary_samples_{0};
};
} // 命名空间 device
