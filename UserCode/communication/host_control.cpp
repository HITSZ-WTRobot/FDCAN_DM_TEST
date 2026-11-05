#include "communication/host_control.hpp"
#include "isr_lock.h"
#include <algorithm>
#include <limits>
namespace host
{
namespace
{
constexpr std::uint16_t code(WireResult r){return static_cast<std::uint16_t>(r);}
std::uint16_t code(device::Result r)
{
    using R=device::Result;
    switch(r){case R::Ok:return 0;case R::Offline:return 1;case R::NotZeroed:return 2;
    case R::NotEnabled:return 3;case R::Busy:return 4;case R::InvalidParameters:return 5;
    case R::OutsideLimits:return 6;case R::NotStationary:return 7;case R::MotorFault:return 8;
    case R::PlanningFailed:return 9;case R::InvalidCommand:return 10;case R::TransmitFailed:return 11;
    case R::LinkFault:return code(WireResult::LinkFault);}return 10;
}
std::uint16_t code(device::M2006Position::Result r)
{
    using R=device::M2006Position::Result;
    switch(r){case R::Ok:return 0;case R::Offline:return 1;case R::NotZeroed:return 2;
    case R::NotEnabled:return 3;case R::Busy:return 4;case R::OutsideLimits:return 6;
    case R::NotStationary:return 7;case R::PlanningFailed:return 9;case R::Fault:return 8;
    case R::InvalidParameters:return 5;case R::TransmitFailed:return 11;
    case R::LinkFault:return code(WireResult::LinkFault);}return 10;
}
std::uint16_t code(device::DMOmniChassis::Result r)
{
    using R=device::DMOmniChassis::Result;
    switch(r){case R::Ok:return 0;case R::Offline:return 1;case R::NotEnabled:return 3;
    case R::Busy:return 4;case R::OutsideLimits:return 6;case R::Fault:return 8;
    case R::LinkFault:return code(WireResult::LinkFault);case R::TransmitFailed:return 11;}return 10;
}
std::uint8_t kind(unsigned i){return i==8?4:(i>=6?3:((i==0||i==1||i==2||i==4)?1:2));}
constexpr float Missing=std::numeric_limits<float>::quiet_NaN();
}
int Control::index(std::uint8_t id)
{
    for(unsigned i=0;i<DeviceIds.size();++i)if(DeviceIds[i]==id)return static_cast<int>(i);
    return -1;
}
std::uint16_t Control::validate(const Frame& r) const
{
    if(r[2]!=WireVersion)return code(WireResult::BadVersion);
    if(r[3]!=0)return code(WireResult::BadType);
    const auto length=get16(r.data()+8);
    if(length>PayloadSize)return code(WireResult::BadLength);
    if(r[10]||r[11])return code(WireResult::BadPadding);
    for(unsigned n=12+length;n<62;++n)if(r[n])return code(WireResult::BadPadding);
    const auto op=static_cast<Opcode>(r[6]);const auto id=r[7];const int i=index(id);
    unsigned expected=0;
    switch(op)
    {
    case Opcode::Heartbeat:case Opcode::Info:
        if(id!=AllMotors)return code(WireResult::UnknownMotor);
        break;
    case Opcode::State:case Opcode::Disable:case Opcode::Stop:
        if(id!=AllMotors&&i<0)return code(WireResult::UnknownMotor);
        break;
    case Opcode::Parameters:case Opcode::Enable:
        if(i<0)return code(WireResult::UnknownMotor);
        break;
    case Opcode::SetZero:
        if(i<0||i==8)return code(WireResult::UnknownMotor);
        break;
    case Opcode::Move:
        if(i<0||i==8)return code(WireResult::UnknownMotor);
        expected=4;
        break;
    case Opcode::SetLimits:
        if(i<0||i>=6)return code(WireResult::UnknownMotor);
        expected=linear(i)?12:20;
        break;
    case Opcode::BodyVelocity:
        if(i!=8)return code(WireResult::UnknownMotor);
        expected=12;
        break;
    case Opcode::Wheels:
        if(i!=8)return code(WireResult::UnknownMotor);
        break;
    default:return code(WireResult::UnknownOpcode);
    }
    return length==expected?0:code(WireResult::BadLength);
}
void Control::linkTick(std::uint32_t now)
{
    ISRGuard guard;
    if(seen_.load()&&!timed_out_.load()&&std::uint32_t(now-last_valid_.load())>=500)
    {
        timed_out_.store(true);
        for(unsigned i=0;i<6;++i)dm_[i]->linkLost();
        chassis_.linkLost();
        for (auto* motor : dji_) motor->linkLost();
    }
}
Frame Control::state(const Frame& r,unsigned i)
{
    Frame f=response(r,DeviceIds[i],0);auto*p=f.data()+12;
    p[4]=kind(i);p[5]=i==8?3:(i<6&&linear(i)?2:1);p[6]=i<6?1:(i==8?2:3);
    if(i<6)
    {
        ISRGuard guard;auto&m=*dm_[i];
        put16(p+2,(m.connected?1:0)|(m.zeroed?2:0)|(m.limits_configured?4:0)|
            (m.enabled?8:0)|(m.enable_pending?16:0)|(m.busy?32:0)|(m.motion_complete?64:0)|
            (m.fault?128:0)|(timedOut()?256:0));
        put16(p+8,code(m.result));put16(p+10,m.motor_state);put32(p+12,m.feedback_count);
        const float scale=linear(i)?device::DMPositionTest::LinearMmPerDegree:1.0f;
        putFloat(p+16,m.position_deg*scale);putFloat(p+20,m.velocity_rpm*6.0f*scale);
        putFloat(p+24,m.accepted_target_deg*scale);putFloat(p+28,m.reference_deg*scale);
        putFloat(p+32,m.reference_velocity_deg_s*scale);putFloat(p+36,m.torque_nm);
        putFloat(p+40,m.commanded_torque_ff_nm);put16(p+44,static_cast<std::uint16_t>(m.command));
    }
    else if(i<8)
    {
        const auto s=dji_[i-6]->feedback();
        put16(p+2,(s.connected?1:0)|(s.zeroed?2:0)|8U*(s.enabled?1:0)|
            (s.busy?32:0)|(s.motion_complete?64:0)|(s.fault?128:0)|(timedOut()?256:0));
        put16(p+8,code(s.result));put16(p+10,0xffff);put32(p+12,s.feedback_count);
        putFloat(p+16,s.position_deg);putFloat(p+20,s.velocity_rpm*6);putFloat(p+24,s.target_deg);
        putFloat(p+28,s.velocity_reference_rpm);finish(f,32);return f;
    }
    else
    {
        const auto s=chassis_.feedback();unsigned connected=0,enabled=0,fault=0;
        for(unsigned n=0;n<4;++n){if(s.wheels[n].connected)connected|=1u<<n;
            if(s.wheels[n].state==motors::DMMotor::State::Enabled)enabled|=1u<<n;
            if(static_cast<unsigned>(s.wheels[n].state)>1)fault|=1u<<n;}
        put16(p+2,(connected==15?1:0)|(s.enabled?8:0)|(s.enable_pending?16:0)|(s.fault?128:0)|(timedOut()?256:0));
        put16(p+8,code(s.result));put16(p+10,0xffff);
        putFloat(p+16,s.velocity_in_body.vx);putFloat(p+20,s.velocity_in_body.vy);putFloat(p+24,s.velocity_in_body.wz);
        putFloat(p+28,s.target.vx);putFloat(p+32,s.target.vy);putFloat(p+36,s.target.wz);
        p[40]=connected;p[41]=enabled;p[42]=fault;finish(f,44);return f;
    }
    finish(f,48);return f;
}
Frame Control::parameters(const Frame&r,unsigned i)
{
    Frame f=response(r,DeviceIds[i],0);auto*p=f.data()+12;
    if(i<6)
    {
        device::MotionLimits l;device::ControlParameters c;bool configured;
        {ISRGuard guard;l=dm_[i]->activeLimits();c=dm_[i]->controlParameters();configured=dm_[i]->limits_configured;}
        put16(p+2,(configured?1:0)|(linear(i)?0:2));p[4]=kind(i);p[5]=linear(i)?2:1;p[6]=1;
        const float scale=linear(i)?device::DMPositionTest::LinearMmPerDegree:1;
        const float values[]{linear(i)?Missing:l.min_deg,linear(i)?Missing:l.max_deg,
            l.max_velocity_deg_s*scale,l.max_acceleration_deg_s2*scale,l.max_jerk_deg_s3*scale,
            c.kp,c.kd,c.torque_ff_nm,c.finish_deg*scale,c.stationary_rpm};
        for(unsigned n=0;n<10;++n)putFloat(p+8+4*n,values[n]);
        finish(f,48);
    }
    else if(i<8)
    {
        const auto c=dji_[i-6]->controlParameters();
        const float values[]{c.velocity_pid.Kp,c.velocity_pid.Ki,c.velocity_pid.Kd,c.velocity_pid.abs_output_max,
            c.position_pd.Kp,c.position_pd.Kd,c.position_pd.abs_output_max,
            c.profile.max_spd,c.profile.max_acc,c.profile.max_jerk,c.finish_deg,c.stationary_rpm};
        for(unsigned n=0;n<12;++n)putFloat(p+2+4*n,values[n]);
        finish(f,50);
    }
    else
    {
        const float values[]{device::DMOmniChassis::MaxVx,device::DMOmniChassis::MaxVy,
            device::DMOmniChassis::MaxWz,device::DMOmniChassis::MaxWheelRpm,
            device::DMOmniChassis::WheelRadiusMm,device::DMOmniChassis::WheelDistanceXmm,device::DMOmniChassis::WheelDistanceYmm};
        for(unsigned n=0;n<7;++n)putFloat(p+2+4*n,values[n]);
        finish(f,30);
    }
    return f;
}
void Control::wheelStates(const Frame&r)
{
    const auto s=chassis_.feedback();pending_count_=4;
    for(unsigned i=0;i<4;++i)
    {
        auto& f=pending_[i];f=response(r,0x41+i,0);auto*p=f.data()+12;
        put16(p+2,s.wheels[i].connected?1:0);p[4]=device::DMOmniChassis::MotorIds[i];p[5]=0x20+p[4];
        put16(p+6,static_cast<std::uint16_t>(s.wheels[i].state));
        putFloat(p+8,s.wheels[i].angle_deg);putFloat(p+12,s.wheels[i].velocity_rpm);
        putFloat(p+16,s.wheels[i].torque_nm);finish(f,20);
    }
}
Frame Control::dispatch(const Frame&r,unsigned i)
{
    const auto op=static_cast<Opcode>(r[6]);
    if(op==Opcode::State)return state(r,i);
    if(op==Opcode::Parameters)return parameters(r,i);
    if(timedOut()&&(op==Opcode::Enable||op==Opcode::Move||op==Opcode::BodyVelocity))
        return response(r,DeviceIds[i],code(WireResult::LinkFault));
    std::uint16_t result=10;
    if(i<6)
    {
        if(op==Opcode::Move)
            result=code(linear(i)?dm_[i]->moveToMm(getFloat(r.data()+12)):
                dm_[i]->execute(device::Command::Move,getFloat(r.data()+12)));
        else
        {
            device::Command cmd=device::Command::None;device::MotionLimits l{};
            switch(op){case Opcode::Enable:cmd=device::Command::Enable;break;
            case Opcode::Disable:cmd=device::Command::Disable;break;case Opcode::Stop:cmd=device::Command::Stop;break;
            case Opcode::SetZero:cmd=device::Command::SetZero;break;
            case Opcode::SetLimits:
                cmd=device::Command::ApplyLimits;
                if(linear(i)){const float s=device::DMPositionTest::LinearMmPerDegree;
                    l.max_velocity_deg_s=getFloat(r.data()+12)/s;l.max_acceleration_deg_s2=getFloat(r.data()+16)/s;l.max_jerk_deg_s3=getFloat(r.data()+20)/s;}
                else l={getFloat(r.data()+12),getFloat(r.data()+16),getFloat(r.data()+20),getFloat(r.data()+24),getFloat(r.data()+28)};
                break;default:break;}
            result=code(dm_[i]->execute(cmd,0,op==Opcode::SetLimits?&l:nullptr));
        }
    }
    else if(i<8)
    {
        auto& m=*dji_[i-6];
        switch(op){case Opcode::Enable:result=code(m.enable());break;case Opcode::Disable:result=code(m.disable());break;
        case Opcode::Stop:result=code(m.stop());break;case Opcode::SetZero:result=code(m.setSoftwareZero());break;
        case Opcode::Move:result=code(m.moveToDegrees(getFloat(r.data()+12)));break;default:break;}
    }
    else
    {
        switch(op){case Opcode::Enable:result=code(chassis_.enable());break;case Opcode::Disable:result=code(chassis_.disable());break;
        case Opcode::Stop:result=code(chassis_.stop());break;
        case Opcode::BodyVelocity:result=code(chassis_.setVelocity({getFloat(r.data()+12),getFloat(r.data()+16),getFloat(r.data()+20)}));break;
        default:break;}
    }
    return response(r,DeviceIds[i],result);
}
void Control::handle(const Frame&r,std::uint32_t now)
{
    pending_index_=0;const auto error=validate(r);
    if(error){pending_[0]=response(r,r[7],error);pending_count_=1;return;}
    if(cached_&&get16(r.data()+4)==get16(cached_request_.data()+4))
    {
        if(r!=cached_request_){pending_[0]=response(r,r[7],code(WireResult::SequenceConflict));pending_count_=1;}
        else{if(!timedOut())last_valid_.store(now);std::copy_n(cached_replies_.begin(),cache_count_,pending_.begin());pending_count_=cache_count_;}
        return;
    }
    const auto op=static_cast<Opcode>(r[6]);
    {ISRGuard guard;
        if(op==Opcode::Heartbeat){timed_out_.store(false);for(unsigned i=0;i<6;++i)dm_[i]->linkRestored();
            chassis_.linkRestored();
            for (auto* motor : dji_) motor->linkRestored();}
        if(!timedOut())last_valid_.store(now);
        seen_.store(true);
    }
    pending_count_=1;
    if(op==Opcode::Heartbeat||op==Opcode::Info)
    {
        pending_[0]=response(r,AllMotors,0);auto&p=pending_[0];auto*d=p.data()+12;
        put16(d+2,1|(timedOut()?2:0));put32(d+4,now);
        if(op==Opcode::Info){put16(d+8,500);d[10]=DeviceIds.size();d[11]=WireVersion;
            for(unsigned i=0;i<DeviceIds.size();++i){d[12+i]=DeviceIds[i];d[21+i]=kind(i);}
            put16(d+30,100);put16(d+32,500);put16(d+34,1000);put16(d+36,500);put32(d+38,115200);finish(p,42);}
        else finish(p,8);
    }
    else if(op==Opcode::Wheels)wheelStates(r);
    else if(r[7]==AllMotors){pending_count_=DeviceIds.size();for(unsigned i=0;i<DeviceIds.size();++i)pending_[i]=dispatch(r,i);}
    else pending_[0]=dispatch(r,static_cast<unsigned>(index(r[7])));
    cached_request_=r;cache_count_=pending_count_;std::copy_n(pending_.begin(),cache_count_,cached_replies_.begin());cached_=true;
}
void Control::drainReplies()
{
    if(pending_index_<pending_count_&&uart_.send(pending_[pending_index_]))++pending_index_;
}
void Control::poll(std::uint32_t now)
{
    uart_.service();if(uart_.takeDiscontinuity())parser_.reset();linkTick(now);drainReplies();
    if(pending_index_==pending_count_)
    {
        std::uint8_t byte;Frame f;
        for(unsigned n=0;n<256&&uart_.pop(byte);++n)
        {
            if(parser_.push(byte,f)){handle(f,now);break;}
        }
        drainReplies();
    }
    for(unsigned i=0;i<6;++i){
        {ISRGuard guard;const auto c=dm_[i]->command;
            if(timedOut()&&(c==device::Command::Enable||c==device::Command::Move||c==device::Command::MoveMm))
                {dm_[i]->command=device::Command::None;dm_[i]->result=device::Result::NotEnabled;}}
        dm_[i]->commandUpdate();}
}
}
