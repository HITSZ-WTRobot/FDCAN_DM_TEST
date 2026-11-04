#pragma once

#include "device/m2006_position.hpp"
#include <array>
#include <cstdint>

namespace device
{
// Dedicated FDCAN3 device group. All motors/observers are registered before start().
// Lifetime is permanent: the bus and watchdog registries retain object addresses.
class M2006Group final
{
public:
    enum class Startup : std::uint32_t
    {
        NotStarted, Ready, FilterFailed, ReceiveCallbackFailed, CanStartFailed
    };

    M2006Group(bsp::can::Can& bus, const M2006Position::Config (&control)[2]);
    M2006Group(const M2006Group&) = delete;
    M2006Group& operator=(const M2006Group&) = delete;

    Startup start(); // Task context only; no automatic enable or recovery.
    void service(); // Existing command task; boot zero processing only.
    void profileUpdate(); // TIM7,100Hz, real trajectory sampling/velocity feedforward.
    void positionUpdate(); // TIM3,500Hz, after the application's sole Watchdog::EatAll().
    void transmitTick(); // TIM6,2kHz input; owns /2 speed PID + one 1kHz aggregate TX.

    // Task-side software command endpoints, no Ozone mailbox or serial stub.
    // Both independent devices expose 0..360deg output-shaft absolute coordinates.
    M2006Position& first() noexcept { return motors_[0]; } // C610 id1=1, feedback0x201.
    M2006Position& second() noexcept { return motors_[1]; } // C610 id1=2, feedback0x202.
    Startup startup() const noexcept { return startup_; }
    std::uint32_t transmitFailures() const;
    std::array<M2006Position::Feedback, 2> feedback();

private:
    void received(const bsp::can::FrameView& frame) noexcept;
    bsp::can::Can& bus_;
    M2006Position motors_[2];
    Startup startup_{Startup::NotStarted};
    bool next_velocity_tick_{false};
    std::uint32_t transmit_failures_{0};
};
} // namespace device
