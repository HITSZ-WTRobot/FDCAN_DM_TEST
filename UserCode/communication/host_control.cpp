#include "communication/host_control.hpp"
#include "isr_lock.h"
#include <algorithm>

namespace host
{
namespace
{
constexpr std::uint16_t code(WireResult r){return static_cast<std::uint16_t>(r);}
bool broadcastAllowed(Opcode op){return op==Opcode::State || op==Opcode::Disable || op==Opcode::Stop;}
}
int Control::motorIndex(std::uint8_t id) const noexcept
{
    return id>=0x0a && id<=0x0f ? id-0x0a : -1;
}
std::uint16_t Control::validate(const Frame& r) const noexcept
{
    if(r[2]!=1)return code(WireResult::BadVersion);
    if(r[3]!=0)return code(WireResult::BadType);
    const auto length=get16(r.data()+8);
    if(length>PayloadSize)return code(WireResult::BadLength);
    if(r[10] || r[11])return code(WireResult::BadPadding);
    for(unsigned i=12+length;i<62;++i)if(r[i])return code(WireResult::BadPadding);
    const auto op=static_cast<Opcode>(r[6]);
    unsigned expected=0;
    switch(op)
    {
    case Opcode::Move:expected=4;break;
    case Opcode::SetLimits:expected=20;break;
    case Opcode::Heartbeat:case Opcode::Info:case Opcode::State:case Opcode::Parameters:
    case Opcode::Enable:case Opcode::Disable:case Opcode::Stop:case Opcode::SetZero:break;
    default:return code(WireResult::UnknownOpcode);
    }
    if(length!=expected)return code(WireResult::BadLength);
    if(op==Opcode::Heartbeat || op==Opcode::Info)
        return r[7]==AllMotors?0:code(WireResult::UnknownMotor);
    if(r[7]==AllMotors && broadcastAllowed(op))return 0;
    return motorIndex(r[7])>=0?0:code(WireResult::UnknownMotor);
}

void Control::linkTick(std::uint32_t now)
{
    ISRGuard guard; // Also called by the task: never reset a curve concurrently with TIM7/TIM6.
    if(seen_.load(std::memory_order_acquire) && !timed_out_.load() &&
       std::uint32_t(now-last_valid_.load(std::memory_order_acquire))>=500)
    {
        timed_out_.store(true,std::memory_order_release);
        for(unsigned i=0;i<6;++i)motors_[i]->linkLost();
    }
}

Frame Control::state(const Frame& r,unsigned i)
{
    Frame reply=response(r,0x0a+i,0);
    {
    ISRGuard guard;
    const auto& m=*motors_[i];auto* p=reply.data()+12;
    const std::uint16_t flags=(m.connected?1:0)|(m.zeroed?2:0)|(m.limits_configured?4:0)|
        (m.enabled?8:0)|(m.enable_pending?16:0)|(m.busy?32:0)|(m.motion_complete?64:0)|
        (m.fault?128:0)|(timedOut()?256:0);
    put16(p+2,flags);put32(p+4,m.motor_state);put32(p+8,m.feedback_count);
    putFloat(p+12,m.position_deg);putFloat(p+16,m.velocity_rpm);putFloat(p+20,m.torque_nm);
    putFloat(p+24,m.accepted_target_deg);putFloat(p+28,m.reference_deg);
    putFloat(p+32,m.reference_velocity_deg_s);putFloat(p+36,m.commanded_torque_ff_nm);
    put16(p+40,static_cast<std::uint16_t>(m.result));
    put16(p+42,static_cast<std::uint16_t>(m.command));
    // Snapshot fields, including flags/result/command, belong to this one short lock.
    }
    finish(reply,44);return reply;
}
Frame Control::parameters(const Frame& r,unsigned i)
{
    Frame reply=response(r,0x0a+i,0);
    device::MotionLimits limits;device::ControlParameters control;bool configured;
    {ISRGuard guard;limits=motors_[i]->activeLimits();control=motors_[i]->controlParameters();configured=motors_[i]->limits_configured;}
    auto* p=reply.data()+12;
    put16(p+2,configured?1:0);
    putFloat(p+4,limits.min_deg);putFloat(p+8,limits.max_deg);
    putFloat(p+12,limits.max_velocity_deg_s);putFloat(p+16,limits.max_acceleration_deg_s2);
    putFloat(p+20,limits.max_jerk_deg_s3);putFloat(p+24,control.kp);putFloat(p+28,control.kd);
    putFloat(p+32,control.torque_ff_nm);putFloat(p+36,control.finish_deg);putFloat(p+40,control.stationary_rpm);
    finish(reply,44);return reply;
}
void Control::dispatch(const Frame& r,unsigned i,Frame& reply)
{
    const auto op=static_cast<Opcode>(r[6]);
    if(op==Opcode::State){reply=state(r,i);return;}
    if(op==Opcode::Parameters){reply=parameters(r,i);return;}
    if(timedOut() && (op==Opcode::Enable || op==Opcode::Move))
    {reply=response(r,0x0a+i,code(WireResult::LinkFault));return;}
    device::Command command=device::Command::None;
    switch(op)
    {
    case Opcode::Enable:command=device::Command::Enable;break;
    case Opcode::Disable:command=device::Command::Disable;break;
    case Opcode::Stop:command=device::Command::Stop;break;
    case Opcode::SetZero:command=device::Command::SetZero;break;
    case Opcode::Move:command=device::Command::Move;break;
    case Opcode::SetLimits:command=device::Command::ApplyLimits;break;
    default:break;
    }
    const device::MotionLimits limits{getFloat(r.data()+12),getFloat(r.data()+16),
        getFloat(r.data()+20),getFloat(r.data()+24),getFloat(r.data()+28)};
    const auto result=motors_[i]->execute(command,op==Opcode::Move?getFloat(r.data()+12):0.0f,
                                         op==Opcode::SetLimits?&limits:nullptr);
    reply=response(r,0x0a+i,static_cast<std::uint16_t>(result));
}
void Control::handle(const Frame& r,std::uint32_t now)
{
    const auto error=validate(r);
    if(error){pending_[0]=response(r,r[7],error);pending_count_=1;pending_index_=0;return;}
    if(cached_ && get16(r.data()+4)==get16(cached_request_.data()+4))
    {
        if(r!=cached_request_){pending_[0]=response(r,r[7],code(WireResult::SequenceConflict));pending_count_=1;}
        else
        {
            if(!timedOut())last_valid_.store(now,std::memory_order_release);
            std::copy_n(cached_replies_.begin(),cache_count_,pending_.begin());pending_count_=cache_count_;
        }
        pending_index_=0;return;
    }
    const auto op=static_cast<Opcode>(r[6]);
    {
        ISRGuard guard;
        // Only a NEW heartbeat clears a host-link timeout. Motor fault latches remain.
        if(op==Opcode::Heartbeat)
        {
            timed_out_.store(false,std::memory_order_release);
            for(unsigned i=0;i<6;++i)motors_[i]->linkRestored();
        }
        if(!timedOut())last_valid_.store(now,std::memory_order_release);
        seen_.store(true,std::memory_order_release);
    }
    pending_index_=0;pending_count_=1;
    if(op==Opcode::Heartbeat || op==Opcode::Info)
    {
        pending_[0]=response(r,AllMotors,0);auto& reply=pending_[0];auto* p=reply.data()+12;
        put16(p+2,(seen_.load()?1:0)|(timedOut()?2:0));put32(p+4,now);
        if(op==Opcode::Info)
        {
            put16(p+8,500);put16(p+10,100);put16(p+12,1000);put16(p+14,2000);
            put16(p+16,500);p[18]=6;p[19]=1;
            for(unsigned i=0;i<6;++i)p[20+i]=0x0a+i;
            for(unsigned i=0;i<6;++i)p[26+i]=0x2a+i;
            finish(reply,32);
        }
        else finish(reply,8);
    }
    else if(r[7]==AllMotors)
    {
        pending_count_=6;
        for(unsigned i=0;i<6;++i)dispatch(r,i,pending_[i]);
    }
    else dispatch(r,static_cast<unsigned>(motorIndex(r[7])),pending_[0]);
    cached_request_=r;std::copy_n(pending_.begin(),pending_count_,cached_replies_.begin());cache_count_=pending_count_;cached_=true;
}
void Control::drainReplies()
{
    if(pending_index_<pending_count_ && uart_.send(pending_[pending_index_]))++pending_index_;
}
void Control::poll(std::uint32_t now)
{
    uart_.service();
    if(uart_.takeDiscontinuity())parser_.reset();
    linkTick(now);
    drainReplies();
    if(pending_index_==pending_count_)
    {
        std::uint8_t byte;Frame request;
        for(unsigned n=0;n<256 && uart_.pop(byte);++n)
            if(parser_.push(byte,request)){handle(request,now);break;}
        drainReplies();
    }
    // Both sources share this task, with existing Ozone mailboxes taking precedence
    // over a serial request for the same motor. A serial execute never overwrites it.
    for(unsigned i=0;i<6;++i)
    {
        {
            ISRGuard guard;
            if(timedOut() && (motors_[i]->command==device::Command::Enable ||
                              motors_[i]->command==device::Command::Move))
            {motors_[i]->command=device::Command::None;motors_[i]->result=device::Result::NotEnabled;}
        }
        motors_[i]->commandUpdate();
    }
}
} // namespace host
