#pragma once
#include "communication/host_uart.hpp"
#include "device/dm_position_test.hpp"
#include <array>
#include <atomic>

namespace host
{
// One task owns parsing, dispatch, reply queue and duplicate cache. TIM3 alone calls
// linkTick; motor IRQs keep their existing schedule. No serial ISR executes motions.
class Control final
{
public:
    Control(Uart& uart, device::DMPositionTest* const* motors) : uart_(uart), motors_(motors) {}
    void poll(std::uint32_t now_ms);
    void linkTick(std::uint32_t now_ms); // 500 Hz; exact >=500ms check, wrap-safe.
    bool timedOut() const noexcept {return timed_out_.load(std::memory_order_acquire);}
    std::uint32_t crcErrors() const noexcept {return parser_.crc_errors();}
private:
    void handle(const Frame& request, std::uint32_t now);
    void dispatch(const Frame& request, unsigned index, Frame& reply);
    Frame state(const Frame& request,unsigned index);
    Frame parameters(const Frame& request,unsigned index);
    std::uint16_t validate(const Frame& request) const noexcept;
    int motorIndex(std::uint8_t id) const noexcept;
    void drainReplies();
    Uart& uart_;
    device::DMPositionTest* const* motors_;
    Parser parser_{};
    std::atomic<bool> seen_{false},timed_out_{false};
    std::atomic<std::uint32_t> last_valid_{0};
    Frame cached_request_{};
    std::array<Frame,6> cached_replies_{};
    std::array<Frame,6> pending_{};
    unsigned cache_count_{0},pending_count_{0},pending_index_{0};
    bool cached_{false};
};
} // namespace host
