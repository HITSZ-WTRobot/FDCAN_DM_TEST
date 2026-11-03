#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace host
{
inline constexpr std::size_t FrameSize = 64;
inline constexpr std::size_t PayloadSize = 50;
inline constexpr std::uint8_t AllMotors = 0xff;
using Frame = std::array<std::uint8_t, FrameSize>;
enum class Opcode : std::uint8_t
{
    Heartbeat=0x01, Info=0x02, State=0x10, Parameters=0x11,
    Enable=0x20, Disable=0x21, Stop=0x22, SetZero=0x23, Move=0x24, SetLimits=0x25
};
enum class WireResult : std::uint16_t
{
    BadVersion=100, BadType=101, BadLength=102, BadPadding=103,
    UnknownOpcode=104, UnknownMotor=105, SequenceConflict=106, LinkFault=107
};
// All multi-byte fields little-endian. CRC16/CCITT-FALSE covers bytes 0..61.
// 0:A5 1:5A 2:version=1 3:type=0 request/1 response 4..5:sequence
// 6:opcode 7:motor CAN ID or FF 8..9:payload length 10..11:reserved=0
// 12..61:payload and zero padding 62..63:CRC little-endian.
std::uint16_t get16(const std::uint8_t* p) noexcept;
std::uint32_t get32(const std::uint8_t* p) noexcept;
float getFloat(const std::uint8_t* p) noexcept;
void put16(std::uint8_t* p, std::uint16_t v) noexcept;
void put32(std::uint8_t* p, std::uint32_t v) noexcept;
void putFloat(std::uint8_t* p, float v) noexcept;
std::uint16_t checksum(std::span<const std::uint8_t> bytes) noexcept;
Frame response(const Frame& request, std::uint8_t motor, std::uint16_t result) noexcept;
void finish(Frame& frame, std::uint16_t payload_length) noexcept;

// Sliding parser: CRC mismatch discards one byte, not a complete presumed frame.
// Partial/gapped frames never execute commands; no assumption about DMA/idle boundaries.
class Parser
{
public:
    bool push(std::uint8_t byte, Frame& complete) noexcept;
    void reset() noexcept { used_=0; }
    std::uint32_t crc_errors() const noexcept { return crc_errors_; }
private:
    Frame bytes_{};
    std::size_t used_{};
    std::uint32_t crc_errors_{};
};
} // namespace host
