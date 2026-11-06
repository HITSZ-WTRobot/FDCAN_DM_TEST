#pragma once
#include "Omni4.hpp"
#include "Master.hpp"
#include "JustEncoder.hpp"
#include "dm.hpp"
#include <array>
#include <cstdint>

namespace device
{
// 常驻 FDCAN2 设备组装。仅提供机体速度控制：实际使用库中的 Master/Omni4，
// 不在应用层实现运动学，也不提供世界坐标位置命令路径。
class DMOmniChassis final
{
public:
    static constexpr std::size_t WheelCount = 4;
    static constexpr std::array<std::uint8_t, WheelCount> MotorIds{0x0b,0x0a,0x09,0x0c};
    static constexpr float WheelRadiusMm=63.5f, WheelDistanceXmm=435.0f, WheelDistanceYmm=435.0f;
    static constexpr float MaxTranslationSpeed=1.0f;
    static constexpr float MaxVx=MaxTranslationSpeed, MaxVy=MaxTranslationSpeed, MaxWz=360.0f, MaxWheelRpm=360.0f;
    struct Config { std::array<bool,WheelCount> reverse{}; };
    enum class Startup : std::uint32_t {NotStarted,Ready,FilterFailed,DelayCompensationFailed,CanStartFailed};
    enum class Result : std::uint16_t {Ok,Offline,NotEnabled,Busy,OutsideLimits,Fault,LinkFault,TransmitFailed};
    struct WheelFeedback
    {
        bool connected{};
        motors::DMMotor::State state{motors::DMMotor::State::Disconnected};
        float angle_deg{},velocity_rpm{},torque_nm{};
    };
    struct Feedback
    {
        std::array<WheelFeedback,WheelCount> wheels{};
        chassis::Velocity velocity_in_body{},target{};
        bool enabled{},enable_pending{},fault{};
        Result result{Result::Ok};
    };
    DMOmniChassis(bsp::can::Can& can,const Config& config);
    DMOmniChassis(const DMOmniChassis&)=delete;
    DMOmniChassis& operator=(const DMOmniChassis&)=delete;
    Startup start();
    Result enable(); // 仅限任务：通过库请求使能，由新反馈确认就绪。
    Result disable();
    Result stop(); // 将机体速度设零，并非位置锁定或急停。
    /// 机体系 m/s、deg/s；合成平移≤1、|wz|≤360、保守轮速包络≤360rpm。
    /// 平移与转动组合超限返回 OutsideLimits 并保留旧目标，不缩放。
    /// 0.4m/s+360deg/s 可接受；1m/s+360deg/s 不可接受。
    Result setVelocity(chassis::Velocity target);
    bool update(); // TIM3，500Hz：四轮参考值/失能帧的唯一周期调度入口。
    void linkLost(); // 可在 ISR 中安全调用，仅请求停止并锁存故障；不额外突发发送 CAN 帧。
    void linkRestored() noexcept { link_blocked_=false; } // 绝不使能或清除故障。
    Feedback feedback();
private:
    bool connected() const;
    void latch(Result reason);
    bsp::can::Can& can_;
    motors::DMMotor motors_[WheelCount];
    controllers::MotorVelController wheel_control_[WheelCount];
    chassis::motion::Omni4 motion_;
    chassis::loc::JustEncoder location_;
    chassis::controller::Master control_;
    Startup startup_{Startup::NotStarted};
    bool running_{false},pending_{false},fault_{false},link_blocked_{false},command_active_{false};
    unsigned enable_ticks_{0};
    chassis::Velocity target_{};
    Result result_{Result::Ok};
};
}
