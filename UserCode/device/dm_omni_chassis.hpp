#pragma once
#include "Omni4.hpp"
#include "Master.hpp"
#include "JustEncoder.hpp"
#include "dm.hpp"
#include <array>
#include <cstdint>

namespace device
{
// Permanent FDCAN2 assembly. Body velocity only: real library Master/Omni4,
// no application kinematics or world-position command path.
class DMOmniChassis final
{
public:
    static constexpr std::size_t WheelCount = 4;
    static constexpr std::array<std::uint8_t, WheelCount> MotorIds{0x0b,0x0a,0x09,0x0c};
    static constexpr float WheelRadiusMm=63.5f, WheelDistanceXmm=435.0f, WheelDistanceYmm=435.0f;
    static constexpr float MaxVx=0.1f, MaxVy=0.1f, MaxWz=10.0f, MaxWheelRpm=30.0f;
    struct Config { std::array<bool,WheelCount> reverse{}; };
    enum class Startup : std::uint32_t {NotStarted,Ready,FilterFailed,DelayCompensationFailed,CanStartFailed};
    enum class Result : std::uint16_t {Ok,Offline,NotEnabled,Busy,OutsideLimits,Fault,LinkFault,TransmitFailed};
    struct WheelFeedback
    {
        bool connected{};
        motors::DMMotor::State state{motors::DMMotor::State::Disconnected};
        float angle_deg{},velocity_rpm{},torque_nm{};
    };
    struct Feedback
    {
        std::array<WheelFeedback,WheelCount> wheels{};
        chassis::Velocity velocity_in_body{},target{};
        bool enabled{},enable_pending{},fault{};
        Result result{Result::Ok};
    };
    DMOmniChassis(bsp::can::Can& can,const Config& config);
    DMOmniChassis(const DMOmniChassis&)=delete;
    DMOmniChassis& operator=(const DMOmniChassis&)=delete;
    Startup start();
    Result enable(); // Task only: library enable requests, fresh feedback confirms readiness.
    Result disable();
    Result stop(); // Zero body speed, NOT a position lock or emergency stop.
    Result setVelocity(chassis::Velocity target);
    bool update(); // TIM3,500Hz: unique periodic owner of all four wheel references/disable frames.
    void linkLost(); // ISR-safe requested stop/fault only; no extra CAN burst.
    void linkRestored() noexcept { link_blocked_=false; } // Never enables/clears fault.
    Feedback feedback();
private:
    bool connected() const;
    void latch(Result reason);
    bsp::can::Can& can_;
    motors::DMMotor motors_[WheelCount];
    controllers::MotorVelController wheel_control_[WheelCount];
    chassis::motion::Omni4 motion_;
    chassis::loc::JustEncoder location_;
    chassis::controller::Master control_;
    Startup startup_{Startup::NotStarted};
    bool running_{false},pending_{false},fault_{false},link_blocked_{false},command_active_{false};
    unsigned enable_ticks_{0};
    chassis::Velocity target_{};
    Result result_{Result::Ok};
};
}
