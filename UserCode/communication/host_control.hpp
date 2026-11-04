#pragma once
#include "communication/host_uart.hpp"
#include "device/dm_position_test.hpp"
#include "device/dm_omni_chassis.hpp"
#include "device/m2006_group.hpp"
#include <array>
#include <atomic>
namespace host
{
// Single task owns parsing/dispatch/replies. Hardware timers keep device control.
class Control final
{
public:
    Control(Uart& uart,device::DMPositionTest* const* dm,device::DMOmniChassis& chassis,
            device::M2006Group& dji):uart_(uart),dm_(dm),chassis_(chassis),dji_(dji){}
    void poll(std::uint32_t now_ms);
    void linkTick(std::uint32_t now_ms); // TIM3 500Hz, also called by task.
    bool timedOut() const noexcept{return timed_out_.load(std::memory_order_acquire);}
    std::uint32_t crcErrors() const noexcept{return parser_.crc_errors();}
private:
    std::uint16_t validate(const Frame&) const;
    static int index(std::uint8_t id);
    static bool linear(unsigned i){return i==0||i==1||i==2||i==4;}
    void handle(const Frame&,std::uint32_t now);
    Frame dispatch(const Frame&,unsigned i);
    Frame state(const Frame&,unsigned i);
    Frame parameters(const Frame&,unsigned i);
    void wheelStates(const Frame&);
    void drainReplies();
    Uart& uart_;
    device::DMPositionTest* const* dm_;
    device::DMOmniChassis& chassis_;
    device::M2006Group& dji_;
    Parser parser_{};
    std::atomic<bool> seen_{false},timed_out_{false};
    std::atomic<std::uint32_t> last_valid_{0};
    Frame cached_request_{};
    std::array<Frame,DeviceIds.size()> cached_replies_{},pending_{};
    unsigned cache_count_{0},pending_count_{0},pending_index_{0};
    bool cached_{false};
};
}
