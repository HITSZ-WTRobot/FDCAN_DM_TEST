#pragma once

#include "dm.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"
#include <cstdint>

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

// Captured at construction; all zero defaults deliberately prevent enable.
// Velocity PID: input rpm, output Nm. Position PD: input deg, output deg/s.
struct ControlParameters
{
    PIDMotor::Config velocity_pid{};
    PD::Config position_pd{};
    float finish_deg{0.0f};
    float stationary_rpm{0.0f};
};

struct MotionLimits
{
    float min_deg{};
    float max_deg{};
    float max_velocity_deg_s{};
    float max_acceleration_deg_s2{};
    float max_jerk_deg_s3{};
};

/** Project-specific composition, following the reference Lift/Flip device structure.
 * No hardware startup, task creation or motor parameter writes in the constructor.
 * The motor, velocity controller and trajectory have the same permanent lifetime.
 * Construction copies fixed gains and arrival tolerances. Changing app configuration
 * after construction does not retune this object: change fixed parameters before Init
 * or rebuild/restart. Only MotionLimits are runtime-configurable via ApplyLimits.
 * Zero gains, zero output caps or zero tolerances inhibit enable; no tuning is assumed.
 * Motor configuration must be MIT; setCurrent carries Nm with MIT Kp=Kd=0. The motor
 * itself must already be configured by the operator; this object never changes mode.
 * Protocol caps come from this motor's Config, not shared J4340 defaults. In particular
 * S3519 position feedback is normalized by the existing driver; reduction_rate remains
 * the external transmission ratio only. Protocol caps are not mechanical safety limits.
 * All ISR callers must share one non-preempting priority. Task operations use short
 * ISRGuard sections; long trajectory solving gates this device's library updates,
 * not CAN reception or the other five devices. Its last bounded torque is held
 * during that short command operation; loss/fault still requests disable.
 */
class DMPositionTest final
{
public:
    DMPositionTest(const motors::DMMotor::Config& motor_config,
                   const ControlParameters& control);
    DMPositionTest(const DMPositionTest&) = delete;
    DMPositionTest& operator=(const DMPositionTest&) = delete;

    void commandUpdate();                 // Ozone polling task; curve solving is here.
    void transmitFailed();               // TX was not submitted; latch and request disable.
    void profileUpdate();                 // TIM7: 100 Hz, dt = 0.01 s.
    void errorUpdate();                   // TIM3: 500 Hz, discrete position PD.
    void controllerUpdate();              // TIM6: 500 Hz, discrete velocity PID + CAN.
    void feedbackReceived() noexcept;     // Called after the driver's receive callback.

    volatile Command command{Command::None};
    volatile Result result{Result::Ok};
    volatile float target_deg{0.0f};       // Absolute relative to this boot's software zero.
    volatile float min_deg{0.0f};          // Staged values; ApplyLimits commits together.
    volatile float max_deg{0.0f};
    volatile float max_velocity_deg_s{0.0f};
    volatile float max_acceleration_deg_s2{0.0f};
    volatile float max_jerk_deg_s3{0.0f};

    volatile float accepted_target_deg{0.0f}; // Last accepted target, not staged Ozone input.
    // Observations, not command inputs. Acceptance, enable feedback and arrival differ.
    volatile bool connected{false};
    volatile bool zeroed{false};
    volatile bool limits_configured{false};
    volatile bool enabled{false};          // Latest valid feedback says enabled.
    volatile bool enable_pending{false};
    volatile bool busy{false};
    volatile bool motion_complete{false};  // Only normal arrival sets true.
    volatile bool fault{false};            // No automatic restart after a fault.
    volatile float position_deg{0.0f};
    volatile float velocity_rpm{0.0f};
    volatile float torque_nm{0.0f};
    volatile float commanded_torque_nm{0.0f};
    volatile std::uint32_t motor_state{0};
    volatile std::uint32_t feedback_count{0};

private:
    [[nodiscard]] bool validControl() const noexcept;
    [[nodiscard]] bool validLimits(const MotionLimits& limits) const noexcept;
    [[nodiscard]] bool inRange(float position) const noexcept;
    void disableOutput(bool latch_fault);
    void observeFeedback();
    [[nodiscard]] Result applyLimits();
    [[nodiscard]] Result setZero();
    [[nodiscard]] Result enableOutput();
    [[nodiscard]] Result move(float target);

    motors::DMMotor motor_;
    controllers::MotorVelController controller_;
    trajectory::MotorTrajectory<1> trajectory_;
    ControlParameters control_;
    float position_lsb_deg_;
    float protocol_max_velocity_deg_s_;
    float protocol_max_torque_nm_;
    MotionLimits limits_{};
    bool command_active_{false};          // Access only under ISRGuard or same-priority ISR.
    bool enable_requested_{false};
    bool ever_connected_{false};
    std::uint32_t observed_feedback_count_{0};
    unsigned enable_wait_ticks_{0};
    unsigned stable_samples_{0};
    float stable_min_{0.0f};
    float stable_max_{0.0f};
};
} // namespace device
