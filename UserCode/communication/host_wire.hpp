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
inline constexpr std::uint8_t WireVersion = 2;
inline constexpr std::uint8_t ChassisDevice = 0x40;
inline constexpr std::array<std::uint8_t,9> DeviceIds{0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x21,0x22,ChassisDevice};
using Frame = std::array<std::uint8_t, FrameSize>;
enum class Opcode : std::uint8_t
{
    Heartbeat=0x01, Info=0x02, State=0x10, Parameters=0x11, Wheels=0x12,
    Enable=0x20, Disable=0x21, Stop=0x22, SetZero=0x23, Move=0x24, SetLimits=0x25,
    BodyVelocity=0x26, MotorVelocity=0x28
};
enum class WireResult : std::uint16_t
{
    BadVersion=100, BadType=101, BadLength=102, BadPadding=103,
    UnknownOpcode=104, UnknownMotor=105, SequenceConflict=106, LinkFault=107
};
// 所有多字节字段均为小端序。CRC16/CCITT-FALSE 覆盖字节 0..61。
// 0:A5 1:5A 2:version=2 3:type=0 请求/1 响应 4..5:sequence（序号）
// 6:opcode（操作码）7:逻辑设备 ID 或 FF（不一定是 CAN ID）
// 8..9:payload length（载荷长度）10..11:reserved=0（保留字段）
// 12..61:payload（载荷）及零填充 62..63:CRC，小端序。
std::uint16_t get16(const std::uint8_t* p) noexcept;
std::uint32_t get32(const std::uint8_t* p) noexcept;
float getFloat(const std::uint8_t* p) noexcept;
void put16(std::uint8_t* p, std::uint16_t v) noexcept;
void put32(std::uint8_t* p, std::uint32_t v) noexcept;
void putFloat(std::uint8_t* p, float v) noexcept;
std::uint16_t checksum(std::span<const std::uint8_t> bytes) noexcept;
Frame response(const Frame& request, std::uint8_t motor, std::uint16_t result) noexcept;
void finish(Frame& frame, std::uint16_t payload_length) noexcept;

// 滑动解析器：CRC 不匹配时只丢弃一个字节，而非整个推定帧。
// 不完整或有间隙的帧绝不执行命令；不假设 DMA/空闲事件边界与帧边界一致。
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
} // 命名空间 host
