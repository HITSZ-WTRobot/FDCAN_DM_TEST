#pragma once

#include "dm.hpp"
#include "s_curve.hpp"
#include <cstdint>
#include <optional>

namespace device
{
enum class Command : std::uint32_t
{
    None = 0, SetZero = 1, Enable = 2, Disable = 3, Move = 4, Stop = 5, ApplyLimits = 6
};

enum class Result : std::uint32_t
{
    Ok = 0, Offline = 1, NotZeroed = 2, NotEnabled = 3, Busy = 4,
    InvalidParameters = 5, OutsideLimits = 6, NotStationary = 7,
    MotorFault = 8, PlanningFailed = 9, InvalidCommand = 10, TransmitFailed = 11
};

// MIT packet coefficients, not MCU PID gains. Copied at construction.
// Low-gain starting values only: no assertion of loaded mechanical stability.
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
    float min_deg{};
    float max_deg{};
    float max_velocity_deg_s{};
    float max_acceleration_deg_s2{};
    float max_jerk_deg_s3{};
};

/** Project-specific six-motor test device: DMMotor + existing SCurveProfile.
 * No external position PD/velocity PID, no hidden hardware startup or heap allocation.
 *
 * Logical positions are output-shaft degrees relative to the software zero. The
 * driver's getAngle() is never reset: zero_output_deg_ preserves its absolute origin
 * so every MIT p_des is an absolute reference in the motor's protocol coordinate.
 * Per the user's tested firmware, S3519 position feedback is pre-reduction and the
 * existing driver normalizes by 19.203. Invert that same normalization for MIT p_des;
 * do not change the library or apply the internal ratio to external reduction_rate.
 * Velocity follows the existing driver's output-shaft feedback contract; MIT v_des
 * uses deg/s, not rpm. Position and velocity thus have separate protocol scales.
 *
 * Command solving runs in the polling task, sampling in TIM7 (100 Hz), feedback and
 * status updates in TIM3 (500 Hz), MIT output in TIM6 (500 Hz). All three timers and
 * CAN RX share one non-preempting priority. Task state handoffs use short ISRGuards;
 * curve solving does not mask interrupts, and the previous reference remains valid.
 *
 * Application-provided limits are staged before CAN starts and automatically
 * validated/applied after the first software zero. Invalid defaults leave
 * limits_configured false and inhibit enable, without clamping or changing PMAX.
 * ApplyLimits still permits an explicit disabled-state override. Subsequent rezero
 * translates accepted limits instead of restoring the initial defaults.
 *
 * Enable first preloads a hold reference while disabled, then requests enable; only
 * fresh feedback confirms state. Faults cancel motion and request disable with no
 * auto-restart. Software limits and protocol bounds do not provide hardware safety.
 */
class DMPositionTest final
{
public:
    DMPositionTest(const motors::DMMotor::Config& motor_config,
                   const ControlParameters& control);
    DMPositionTest(const DMPositionTest&) = delete;
    DMPositionTest& operator=(const DMPositionTest&) = delete;

    void commandUpdate();             // Ozone task: commands and new S-curve solving.
    void profileUpdate();             // TIM7: sample position + velocity, dt=0.01 s.
    void feedbackUpdate();            // TIM3: new feedback, readiness/faults/arrival.
    void sendUpdate();                // TIM6: one MIT/control frame per motor.
    void transmitFailed();            // Called when no frame reached the hardware FIFO.
    void feedbackReceived() noexcept; // Called after the driver's receive callback.

    volatile Command command{Command::None};
    volatile Result result{Result::Ok};
    volatile float target_deg{0.0f};
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
    [[nodiscard]] bool validLimits(const MotionLimits& limits) const noexcept;
    [[nodiscard]] bool inRange(float position) const noexcept;
    [[nodiscard]] bool validReference(float position, float velocity) const noexcept;
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
    bool ever_connected_{false};
    EnablePhase enable_phase_{EnablePhase::None};
    std::uint32_t preload_feedback_count_{0};
    std::uint32_t observed_feedback_count_{0};
    unsigned enable_wait_ticks_{0};
    unsigned stable_samples_{0};
    float stable_min_{0.0f};
    float stable_max_{0.0f};
};
} // namespace device
