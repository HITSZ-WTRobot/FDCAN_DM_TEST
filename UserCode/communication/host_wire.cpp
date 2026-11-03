#include "communication/host_wire.hpp"
#include "crc.hpp"
#include <cstring>
#include <limits>
namespace host
{
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
using Crc = crc::CRCX<16,0x1021,0xffff,false,false,0>;
std::uint16_t get16(const std::uint8_t* p) noexcept {return p[0]|(std::uint16_t(p[1])<<8);}
std::uint32_t get32(const std::uint8_t* p) noexcept {return get16(p)|(std::uint32_t(get16(p+2))<<16);}
float getFloat(const std::uint8_t* p) noexcept {const auto bits=get32(p);float f;std::memcpy(&f,&bits,4);return f;}
void put16(std::uint8_t* p,std::uint16_t v) noexcept {p[0]=v;p[1]=v>>8;}
void put32(std::uint8_t* p,std::uint32_t v) noexcept {put16(p,v);put16(p+2,v>>16);}
void putFloat(std::uint8_t* p,float v) noexcept {std::uint32_t bits;std::memcpy(&bits,&v,4);put32(p,bits);}
std::uint16_t checksum(std::span<const std::uint8_t> bytes) noexcept {return Crc::calc(bytes.data(),bytes.size());}
void finish(Frame& f,std::uint16_t length) noexcept
{
    put16(f.data()+8,length);
    put16(f.data()+62,checksum(std::span(f).first(62)));
}
Frame response(const Frame& r,std::uint8_t motor,std::uint16_t result) noexcept
{
    Frame f{};f[0]=0xa5;f[1]=0x5a;f[2]=1;f[3]=1;
    f[4]=r[4];f[5]=r[5];f[6]=r[6];f[7]=motor;
    put16(f.data()+12,result);finish(f,2);return f;
}
bool Parser::push(std::uint8_t byte,Frame& complete) noexcept
{
    bytes_[used_++]=byte;
    while(used_ && bytes_[0]!=0xa5){std::memmove(bytes_.data(),bytes_.data()+1,--used_);}
    if(used_>=2 && bytes_[1]!=0x5a){std::memmove(bytes_.data(),bytes_.data()+1,--used_);return false;}
    if(used_!=FrameSize)return false;
    if(get16(bytes_.data()+62)==checksum(std::span(bytes_).first(62)))
    {complete=bytes_;used_=0;return true;}
    ++crc_errors_;
    std::memmove(bytes_.data(),bytes_.data()+1,--used_);
    return false;
}
} // namespace host
