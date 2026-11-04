#include "device/m2006_group.hpp"
#include "isr_lock.h"

namespace device
{
namespace
{
motors::DJIMotor::Config motorConfig(bsp::can::Can& bus, std::uint8_t id)
{
    return {.can = bus, .type = motors::DJIMotor::Type::M2006_C610, .id1 = id,
            .auto_zero = false, // Device confirms new stationary/stable samples before first zero.
            .reverse = false, .reduction_rate = 1.0f}; // Internal36:1 belongs to DJI driver.
}
}

M2006Group::M2006Group(bsp::can::Can& bus, const M2006Position::Config (&control)[2]) :
    bus_(bus), motors_{M2006Position{motorConfig(bus, 1), control[0]},
                      M2006Position{motorConfig(bus, 2), control[1]}}
{
}

M2006Group::Startup M2006Group::start()
{
    if (startup_ != Startup::NotStarted)
        return startup_;
    if (motors::DJIMotor::CAN_FilterInit(bus_, 0) != bsp::can::Status::Ok ||
        bus_.configure_filter(bsp::can::GlobalFilter{
            .reject_standard_remote = true, .reject_extended_remote = true}) != bsp::can::Status::Ok)
        return startup_ = Startup::FilterFailed;
    // Registration order guarantees the matching DJIMotor decoded this frame first.
    if (bus_.register_callback<&M2006Group::received>(*this) != bsp::can::Status::Ok)
        return startup_ = Startup::ReceiveCallbackFailed;
    if (bus_.start() != bsp::can::Status::Ok)
        return startup_ = Startup::CanStartFailed;
    return startup_ = Startup::Ready;
}

void M2006Group::received(const bsp::can::FrameView& frame) noexcept
{
    if (frame.header.id_type != bsp::can::IdType::Standard ||
        frame.header.format != bsp::can::FrameFormat::Classic ||
        frame.header.type != bsp::can::FrameType::Data || frame.data.size() != 8)
        return;
    if (frame.header.id >= 0x201 && frame.header.id <= 0x202)
        motors_[frame.header.id - 0x201].feedbackReceived();
}

void M2006Group::service()
{
    if (startup_ != Startup::Ready)
        return;
    for (auto& motor : motors_)
        motor.service();
}

void M2006Group::profileUpdate()
{
    if (startup_ != Startup::Ready)
        return;
    for (auto& motor : motors_)
        motor.profileUpdate();
}

void M2006Group::positionUpdate()
{
    if (startup_ != Startup::Ready)
        return;
    for (auto& motor : motors_)
        motor.positionUpdate();
}

void M2006Group::transmitTick()
{
    if (startup_ != Startup::Ready)
        return;
    next_velocity_tick_ = !next_velocity_tick_;
    if (next_velocity_tick_)
        return;
    for (auto& motor : motors_)
        motor.velocityUpdate();
    // One owner, one attempt per 1ms; slots3/4 are zero, group5..8 is not transmitted.
    const auto sent = motors::DJIMotor::SendIqCommand(
        bus_, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
    if (sent != bsp::can::SendResult::Submitted)
    {
        ++transmit_failures_;
        for (auto& motor : motors_)
            motor.transmitFailed(); // Zero cache/latch only; no extra submit or immediate retry.
    }
}

std::uint32_t M2006Group::transmitFailures() const
{
    ISRGuard guard;
    return transmit_failures_;
}

std::array<M2006Position::Feedback, 2> M2006Group::feedback()
{
    ISRGuard guard;
    return {motors_[0].feedback(), motors_[1].feedback()};
}
} // namespace device
