#pragma once

#include "dji.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include <cstdint>
#include <optional>

namespace device
{
/** One M2006/C610, using the library's output-shaft deg/rpm coordinates.
 * Construct before Can::start(); the borrowed bus and this object must be permanent.
 * Commands/service have one task consumer. Feedback and all periodic callbacks must
 * share a non-preempting IRQ priority. Short ISRGuards protect task handoffs and
 * snapshots; command_active_ gates trajectory access while task-side solving/reset
 * runs with interrupts enabled. No method submits a CAN frame or feeds watchdogs.
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
        bool enabled{false}; // Software controller state, not C610 enable confirmation.
        bool busy{false};
        bool motion_complete{false};
        bool fault{false};
        float position_deg{0.0f};
        float velocity_rpm{0.0f};
        float target_deg{0.0f};
        // Last actual speed-PID reference: trajectory velocity FF + position PD.
        float velocity_reference_rpm{0.0f};
        std::uint32_t feedback_count{0};
    };

    M2006Position(const motors::DJIMotor::Config& motor_config, const Config& config);
    M2006Position(const M2006Position&) = delete;
    M2006Position& operator=(const M2006Position&) = delete;
    M2006Position(M2006Position&&) = delete;
    M2006Position& operator=(M2006Position&&) = delete;

    // Task only. Stop holds position; disable releases control and caches zero current.
    Result enable();
    Result disable();
    Result stop();
    Result moveToDegrees(float target_deg); // Finite, inclusive [0, 360], never clamped.
    // Disabled + 50 fresh stable samples; explicitly clears a latched fault on success.
    Result setSoftwareZero();
    void service(); // First boot auto-zero and deferred fault-history reset; never enables.

    void feedbackReceived() noexcept; // Observer runs after the valid raw DJI callback.
    void profileUpdate();             // 100 Hz: real profileUpdate(0.01f).
    void positionUpdate();            // 500 Hz: fresh-sample observation + real errorUpdate.
    void velocityUpdate();            // 1 kHz: real controllerUpdate; cached output only.
    void transmitFailed();            // Latches fault; no immediate CAN submission.
    [[nodiscard]] Feedback feedback() const;

private:
    using Trajectory = trajectory::MotorTrajectory<1>;
    static motors::DJIMotor::Config manualZeroConfig(const motors::DJIMotor::Config& config);
    [[nodiscard]] bool validParameters(const motors::DJIMotor::Config& config) const;
    [[nodiscard]] bool inBounds(float position) const;
    [[nodiscard]] bool stableNow() const;
    void resetStability();
    void observeConnection(); // Caller owns IRQ context or ISRGuard.
    void latchFault(Result reason, bool invalidate_zero = false);
    Result recordResult(Result result);
    // Task only, with command_active_ set; construction/SCurve work stays unmasked.
    void rebuildTrajectory();

    motors::DJIMotor motor_;
    controllers::MotorVelController controller_;
    const Config config_;
    const bool parameters_valid_;
    const float encoder_tolerance_deg_; // Two real output encoder LSB, including 36:1.
    std::optional<Trajectory> trajectory_;
    Feedback state_{};
    volatile bool command_active_{false};
    bool reset_pending_{false};
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
} // namespace device
