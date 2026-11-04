#pragma once

#include "Omni4.hpp"
#include "dm.hpp"
#include <array>
#include <cstdint>

namespace device
{
// Dedicated FDCAN2 assembly, not a chassis Controller or a communication endpoint.
// Order follows Omni4: front-right, front-left, rear-left, rear-right.
// Permanent lifetime: CAN callbacks and the shared watchdog registry retain motor addresses.
class DMOmniChassis final
{
public:
    static constexpr std::size_t WheelCount = 4;
    static constexpr std::array<std::uint8_t, WheelCount> MotorIds{0x0b, 0x0a, 0x09, 0x0c};
    static constexpr float WheelRadiusMm = 63.5f;
    static constexpr float WheelDistanceXmm = 435.0f;
    static constexpr float WheelDistanceYmm = 435.0f;

    struct Config
    {
        // User-selected initial directions; not physically verified.
        // Copied on construction. S3519's internal reduction is handled by DM.
        std::array<bool, WheelCount> reverse{};
    };

    enum class Startup : std::uint32_t
    {
        NotStarted, Ready, FilterFailed, DelayCompensationFailed, CanStartFailed
    };

    struct WheelFeedback
    {
        bool connected{};
        motors::DMMotor::State state{motors::DMMotor::State::Disconnected};
        float angle_deg{};
        float velocity_rpm{};
        float torque_nm{};
    };
    struct Feedback
    {
        std::array<WheelFeedback, WheelCount> wheels{};
        chassis::Velocity velocity_in_body{}; // m/s, m/s, deg/s; meaningful only with fresh wheel feedback.
    };

    DMOmniChassis(bsp::can::Can& can, const Config& config);
    DMOmniChassis(const DMOmniChassis&) = delete;
    DMOmniChassis& operator=(const DMOmniChassis&) = delete;

    // Call after construction, before pollDisabled(). Does not enable any motor.
    Startup start();
    // Structural bring-up only: four disable packets, one attempt/wheel, never enable.
    // Existing TIM3 calls this at 500 Hz after the unique Watchdog::EatAll().
    // This disabled polling must be replaced when a real Controller is connected.
    bool pollDisabled();
    Feedback feedback();

    // Bind the library's real Controller here when adding a control input.
    chassis::motion::Omni4& motion() noexcept { return motion_; }

private:
    bsp::can::Can& can_; // Dedicated bus; must outlive the entire assembly.
    motors::DMMotor motors_[WheelCount];
    controllers::MotorVelController wheel_control_[WheelCount];
    chassis::motion::Omni4 motion_;
    Startup startup_{Startup::NotStarted};
};
} // namespace device
