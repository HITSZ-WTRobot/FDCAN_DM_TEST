#include "device/dm_omni_chassis.hpp"
#include "isr_lock.h"
#include "fdcan.h"
#include <cmath>
namespace device
{
namespace
{
// 合成平移速度限制为1m/s；任意方向叠加10deg/s旋转的轮速包络小于160rpm。
// 这里只核对包络，轮速分解仍使用原Omni4库，不另外实现运动学。
static_assert(DMOmniChassis::MaxTranslationSpeed+
              0.435f*0.707106782f*DMOmniChassis::MaxWz*3.141592654f/180.0f <
              DMOmniChassis::MaxWheelRpm*2.0f*3.141592654f*0.0635f/60.0f);
motors::DMMotor::Config motorConfig(bsp::can::Can& can,std::size_t i,const DMOmniChassis::Config& config)
{
    return {.can=can,.id0=DMOmniChassis::MotorIds[i],.type=motors::DMMotor::Type::S3519,
            .mode=motors::DMMotor::Mode::Vel,.pos_max_rad=12.5f,.vel_max_rad=200.0f,.tor_max=10.0f,
            .auto_zero=false,.reverse=config.reverse[i],.reduction_rate=1.0f,.can_fd_brs=true};
}
constexpr controllers::MotorVelController::Config WheelControl{
    .ctrl_mode=controllers::ControlMode::InternalVel,.internal_set_ratio=1};
}
DMOmniChassis::DMOmniChassis(bsp::can::Can& can,const Config& cfg):
    can_(can),motors_{motors::DMMotor{motorConfig(can,0,cfg)},motors::DMMotor{motorConfig(can,1,cfg)},
                     motors::DMMotor{motorConfig(can,2,cfg)},motors::DMMotor{motorConfig(can,3,cfg)}},
    wheel_control_{controllers::MotorVelController{&motors_[0],WheelControl},
                   controllers::MotorVelController{&motors_[1],WheelControl},
                   controllers::MotorVelController{&motors_[2],WheelControl},
                   controllers::MotorVelController{&motors_[3],WheelControl}},
    motion_({.wheel_radius=WheelRadiusMm,.wheel_distance_x=WheelDistanceXmm,.wheel_distance_y=WheelDistanceYmm,
             .wheel_front_right=&wheel_control_[0],.wheel_front_left=&wheel_control_[1],
             .wheel_rear_left=&wheel_control_[2],.wheel_rear_right=&wheel_control_[3]}),
    location_(motion_),control_(motion_,location_,{}) // 不暴露姿态/轨迹 API。
{
}
DMOmniChassis::Startup DMOmniChassis::start()
{
    if(startup_!=Startup::NotStarted)return startup_;
    if(motors::DMMotor::CAN_FilterInit(can_,0)!=bsp::can::Status::Ok ||
       can_.configure_filter(bsp::can::GlobalFilter{.reject_standard_remote=true,.reject_extended_remote=true})!=bsp::can::Status::Ok)
        return startup_=Startup::FilterFailed;
    const auto sample=hfdcan2.Init.DataPrescaler*(1U+hfdcan2.Init.DataTimeSeg1);
    if(can_.configure_tx_delay_compensation({true,sample,0})!=bsp::can::Status::Ok)
        return startup_=Startup::DelayCompensationFailed;
    if(can_.start()!=bsp::can::Status::Ok)return startup_=Startup::CanStartFailed;
    return startup_=Startup::Ready;
}
bool DMOmniChassis::connected() const
{
    for(const auto& m:motors_)if(!m.isConnected())return false;
    return true;
}
void DMOmniChassis::latch(Result reason)
{
    running_=pending_=false;target_={};fault_=true;result_=reason;
}
DMOmniChassis::Result DMOmniChassis::enable()
{
    {
        ISRGuard guard;
        if(command_active_||pending_)return Result::Busy;
        if(link_blocked_)return Result::LinkFault;
        if(startup_!=Startup::Ready||!connected())return Result::Offline;
        if(running_)return Result::Ok;
        for(const auto& m:motors_)
            if(m.state()!=motors::DMMotor::State::Disabled||std::fabs(m.getVelocity())>1.1f)
                return Result::Busy;
        command_active_=true;
    }
    // 库仅在任务上下文中获取互斥锁。此处发送四个使能
    // 请求；常规参考值及所有失能请求仍在 update() 中发送。
    const bool accepted=control_.enable();
    control_.hold();
    ISRGuard guard;
    command_active_=false;
    if(link_blocked_||!connected()) {latch(link_blocked_?Result::LinkFault:Result::Offline);return result_;}
    if(!accepted){latch(Result::Fault);return result_;}
    running_=true;pending_=true;fault_=false;enable_ticks_=10;target_={};return result_=Result::Ok;
}
DMOmniChassis::Result DMOmniChassis::disable()
{
    ISRGuard guard;
    if(command_active_)return Result::Busy;
    running_=pending_=false;target_={};
    control_.disable(); // 释放软件控制；显式失能帧延后至 update() 发送。
    return result_=Result::Ok;
}
DMOmniChassis::Result DMOmniChassis::setVelocity(chassis::Velocity target)
{
    {
        ISRGuard guard;
        if(command_active_||pending_)return Result::Busy;
        if(link_blocked_)return Result::LinkFault;
        if(!connected())return Result::Offline;
        if(!running_||fault_)return Result::NotEnabled;
        if(!std::isfinite(target.vx)||!std::isfinite(target.vy)||!std::isfinite(target.wz)||
           std::fabs(target.vx)>MaxVx||std::fabs(target.vy)>MaxVy||std::fabs(target.wz)>MaxWz||
           std::hypot(target.vx,target.vy)>MaxTranslationSpeed+1.0e-6f)
            return Result::OutsideLimits;
        command_active_=true;
    }
    control_.setVelocityInBody(target,false); // 库的实际运动学计算在下一个 500Hz 周期执行。
    ISRGuard guard;
    command_active_=false;
    if(link_blocked_||!connected()||!running_||fault_)
        return result_;
    target_=target;return result_=Result::Ok;
}
DMOmniChassis::Result DMOmniChassis::stop()
{
    return setVelocity({});
}
void DMOmniChassis::linkLost()
{
    ISRGuard guard;link_blocked_=true;latch(Result::LinkFault);
}
bool DMOmniChassis::update()
{
    if(startup_!=Startup::Ready||command_active_)return false;
    if(running_)
    {
        if(!connected())latch(Result::Offline);
        else
        {
            bool all_enabled=true;
            for(const auto& m:motors_)
            {
                const auto s=m.state();
                if(s!=motors::DMMotor::State::Enabled)all_enabled=false;
                if(static_cast<unsigned>(s)>1)latch(Result::Fault);
            }
            if(pending_)
            {
                if(all_enabled)pending_=false;
                else if(enable_ticks_--==0)latch(Result::Fault);
            }
            else if(!all_enabled)latch(Result::Fault);
        }
    }
    const auto before=can_.counters().tx_submitted_frames;
    if(running_&&!fault_)
    {
        location_.update(0.002f);
        control_.controllerUpdate(); // setRef 每轮发送一次；绝不能再调用 motion_.update()。
    }
    else
    {
        control_.disable();
        for(auto& m:motors_)m.disable();
    }
    const bool ok=can_.counters().tx_submitted_frames-before==WheelCount;
    if(!ok)latch(Result::TransmitFailed);
    return ok;
}
DMOmniChassis::Feedback DMOmniChassis::feedback()
{
    ISRGuard guard;Feedback f{};
    for(std::size_t i=0;i<WheelCount;++i)
        f.wheels[i]={motors_[i].isConnected(),motors_[i].state(),motors_[i].getAngle(),motors_[i].getVelocity(),motors_[i].getTorque()};
    f.velocity_in_body=motion_.forwardGetVelocity();f.target=target_;
    f.enabled=running_;f.enable_pending=pending_;f.fault=fault_;f.result=result_;return f;
}
}
