#pragma once

#include "dm.hpp"
#include "s_curve.hpp"
#include <cstdint>
#include <optional>

namespace device
{
enum class Command : std::uint32_t
{
    None = 0, SetZero = 1, Enable = 2, Disable = 3, Move = 4, Stop = 5, ApplyLimits = 6,
    MoveMm = 7
};

enum class Result : std::uint32_t
{
    Ok = 0, Offline = 1, NotZeroed = 2, NotEnabled = 3, Busy = 4,
    InvalidParameters = 5, OutsideLimits = 6, NotStationary = 7,
    MotorFault = 8, PlanningFailed = 9, InvalidCommand = 10, TransmitFailed = 11
};

enum class PositionKind { Joint, LinearActuator };

struct LinearFeedback
{
    float position_mm;
    float velocity_mm_s;
    float accepted_target_mm;
    float reference_mm;
    float reference_velocity_mm_s;
};

// MIT 报文系数，并非 MCU PID 增益；在构造时复制。
// 仅为低增益初始值，不保证带载机械稳定性。
struct ControlParameters
{
    float kp{5.0f};
    float kd{0.5f};
    float torque_ff_nm{0.0f};
    float finish_deg{0.3f};
    float stationary_rpm{0.1f};
};

struct MotionLimits
{
    float min_deg{}; // 仅用于关节软件限位区间，直线行程忽略此项。
    float max_deg{};
    float max_velocity_deg_s{};
    float max_acceleration_deg_s2{};
    float max_jerk_deg_s3{};
};

/** 工程专用的六电机测试设备：DMMotor + 现有 SCurveProfile。
 * 无外部位置 PD/速度 PID，不隐式启动硬件或分配堆内存。
 *
 * 逻辑位置以输出轴相对软件零点的角度表示，单位为度。
 * 驱动的 getAngle() 从不复位：zero_output_deg_ 保留其绝对原点，
 * 因而每个 MIT p_des 都是电机协议坐标系中的绝对参考值。
 * 根据用户实测的固件，S3519 位置反馈为减速前数值，
 * 现有驱动按 19.203 进行归一化。MIT p_des 需逆向应用同一归一化；
 * 不修改库，也不将内置减速比应用于外部 reduction_rate。
 * 速度遵循现有驱动的输出轴反馈契约；MIT v_des
 * 使用 deg/s，而非 rpm。因此位置与速度使用不同的协议缩放比例。
 *
 * 命令求解在轮询任务中执行，TIM7（100 Hz）采样，
 * TIM3（500 Hz）更新反馈和状态。TIM6 以 2 kHz 交替调度两组，
 * 每组三台电机；每台电机保留 1 kHz 的发送时隙。这三个定时器与
 * CAN 接收共用同一不可相互抢占的优先级。任务状态交接使用短时 ISRGuard；
 * 曲线求解不屏蔽中断，期间上一参考值仍然有效。
 *
 * 关节行程限制为每次新软件零点两侧各 180 度。直线
 * 执行器没有软件行程区间，但仍须满足有限数值及绝对协议
 * 编码范围约束。若关节完整限位区间无法编码，则拒绝设置新零点。
 * 两类设备均保留已校验的轨迹速度、加速度和加加速度约束。
 *
 * 使能时先在失能状态下预装保持参考值，再请求使能；仅
 * 新反馈可确认状态。故障会取消运动并请求失能，不
 * 自动重启。软件限位和协议范围约束不提供硬件安全保障。
 * 仅 sendUpdate 路径提交 CAN 帧，每次调用最多尝试一次。
 * 命令、反馈/轨迹故障和 transmitFailed 仅改变请求状态；
 * 失能帧在下一个分配的时隙提交，不在时隙外突发发送。
 */
class DMPositionTest final
{
public:
    DMPositionTest(const motors::DMMotor::Config& motor_config,
                   const ControlParameters& control, PositionKind kind);
    DMPositionTest(const DMPositionTest&) = delete;
    DMPositionTest& operator=(const DMPositionTest&) = delete;

    void commandUpdate();             // 共用 UART/Ozone 任务：处理命令及新 S 曲线求解。
    void profileUpdate();             // TIM7：采样位置与速度，dt=0.01 s。
    void feedbackUpdate();            // TIM3：新反馈、就绪状态、故障和到位判定。
    void sendUpdate();                // TIM6：仅调度选中组，每 1 ms 尝试发送一帧。
    void transmitFailed();            // 锁存故障，不立即重试或提交 CAN 帧。
    void feedbackReceived() noexcept; // 在驱动接收回调之后调用。

    // 共用的任务侧命令路径：已有 Ozone 命令时拒绝新命令，绝不覆盖原命令。
    Result execute(Command requested, float target = 0.0f, const MotionLimits* limits = nullptr);
    void linkLost(); // 可在 ISR 中安全请求状态变更；CAN 失能仍使用分配的发送时隙。
    void linkRestored() noexcept { host_link_blocked_ = false; } // 不自动使能。
    ControlParameters controlParameters() const noexcept { return control_; }
    MotionLimits activeLimits() const noexcept { return limits_; }
    // 任务侧 API；使用同一命令消费者，绝不能从 ISR 调用。
    // 命令仅改变状态；所有 CAN 输出仍使用现有 TIM6 时隙。
    static constexpr float LinearMmPerRevolution = 226.2f; // 用户提供的输出轴每转标称行程。
    static constexpr float LinearMmPerDegree = LinearMmPerRevolution / 360.0f;
    Result moveToMm(float target_mm); // 相对当前软件零点的绝对位置，仅适用于直线执行器。
    Result setSoftwareZero(); // 须失能且反馈稳定；成功后关节限位区间以新零点为中心。
    std::optional<LinearFeedback> linearFeedback() const; // 关节返回 nullopt，不分配堆内存。

    volatile Command command{Command::None};
    volatile Result result{Result::Ok};
    volatile float target_deg{0.0f}; // 关节 Move 输入，同时保留内部角度参考记录。
    volatile float target_mm{0.0f}; // 直线 MoveMm 输入；原始 Move/target_deg 仅适用于关节。
    volatile float min_deg{0.0f};
    volatile float max_deg{0.0f};
    volatile float max_velocity_deg_s{0.0f};
    volatile float max_acceleration_deg_s2{0.0f};
    volatile float max_jerk_deg_s3{0.0f};

    volatile float accepted_target_deg{0.0f};
    volatile bool connected{false};
    volatile bool zeroed{false};
    volatile bool limits_configured{false};
    volatile bool enabled{false};
    volatile bool enable_pending{false};
    volatile bool busy{false};
    volatile bool motion_complete{false};
    volatile bool fault{false};
    volatile float position_deg{0.0f};
    volatile float velocity_rpm{0.0f};
    volatile float torque_nm{0.0f};
    volatile float reference_deg{0.0f};
    volatile float reference_velocity_deg_s{0.0f};
    volatile float commanded_torque_ff_nm{0.0f};
    volatile std::uint32_t motor_state{0};
    volatile std::uint32_t feedback_count{0};

private:
    enum class EnablePhase { None, Preload, WaitPreload, SendEnable, WaitEnable };
    [[nodiscard]] bool validControl() const noexcept;
    [[nodiscard]] bool validLimits(const MotionLimits& limits, float zero) const noexcept;
    [[nodiscard]] bool inRange(float position) const noexcept;
    [[nodiscard]] bool validReference(float position, float velocity) const noexcept;
    [[nodiscard]] bool validAbsoluteReference(float position, float velocity) const noexcept;
    [[nodiscard]] float currentPosition() const noexcept;
    void disableOutput(bool latch_fault);
    void holdPosition(float position) noexcept;
    void sendReference(float torque_ff);
    [[nodiscard]] Result applyLimits();
    [[nodiscard]] Result setZero();
    [[nodiscard]] Result enableOutput();
    [[nodiscard]] Result move(float target);

    motors::DMMotor motor_;
    ControlParameters control_;
    const PositionKind kind_;
    MotionLimits limits_{};
    std::optional<velocity_profile::SCurveProfile> profile_{};
    float position_scale_;
    float velocity_scale_;
    float protocol_max_position_deg_;
    float protocol_max_velocity_deg_s_;
    float protocol_max_torque_nm_;
    float position_lsb_deg_;
    float zero_output_deg_{0.0f};
    float elapsed_s_{0.0f};
    float duration_s_{0.0f};
    bool command_active_{false};
    bool enable_requested_{false};
    bool host_link_blocked_{false};
    bool ever_connected_{false};
    EnablePhase enable_phase_{EnablePhase::None};
    std::uint32_t preload_feedback_count_{0};
    std::uint32_t observed_feedback_count_{0};
    unsigned enable_wait_ticks_{0};
    unsigned stable_samples_{0};
    float stable_min_{0.0f};
    float stable_max_{0.0f};
};
} // 命名空间 device
