#pragma once

#include "device/dm_omni_chassis.hpp"
#include "device/dm_position_test.hpp"
#include "device/m2006_position.hpp"
#include <array>
#include <cstdint>

// 工程参数来源。各常驻设备构造时复制这些值；
// 此后修改这些变量并不是在线调参 API。
// HAL 句柄、对象所有权、任务或硬件初始化均不应放在此处。
namespace config
{
struct DMMotorParameters
{
    std::uint8_t can_id;
    motors::DMMotor::Type type;
    float pos_max_rad;
    float vel_max_rad_s;
    float torque_max_nm;
    bool reverse{false};
    float reduction_rate{1.0f}; // 仅表示外接减速；内部归一化由现有驱动处理。
};

// 来源：已批准的 DM 参数图片；这些是映射量程，不是机械行程。
inline constexpr std::array<DMMotorParameters, 6> dm_motor_parameters{{
    {0x0a, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // 右前直线执行器。
    {0x0b, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // 中部直线执行器。
    {0x0c, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // 后部直线执行器。
    {0x0d, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}, // 左前关节。
    {0x0e, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // 左前直线执行器。
    {0x0f, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}  // 右前关节。
}};

// 现有低增益 MIT 初始值，并非带载整定值，也不是 MCU PID 增益。
inline device::ControlParameters dm_control[6]{
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=1.1f},
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=1.1f},
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=1.1f},
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=0.1f},
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=1.1f},
    {.kp=5.0f, .kd=0.5f, .torque_ff_nm=0.0f, .finish_deg=0.3f, .stationary_rpm=0.1f}
};
inline constexpr device::MotionLimits joint_motion_limits{
    -180.0f, 180.0f, 10.0f, 20.0f, 100.0f
};
inline constexpr device::MotionLimits linear_motion_dynamics{
    .max_velocity_deg_s=10.0f, .max_acceleration_deg_s2=20.0f, .max_jerk_deg_s3=100.0f
}; // 不设直线软件行程区间；mm 换算仍由设备 API 处理。

// FR/FL/RL/RR 方向在构造时复制；初始正方向由用户选择。
inline device::DMOmniChassis::Config chassis{};

struct M2006MotorParameters
{
    std::uint8_t id;
    bool reverse;
    float reduction_rate; // 仅表示外接减速；DJI 处理内部 36:1 减速。
};
inline constexpr std::array<M2006MotorParameters, 2> m2006_motor_parameters{{
    {1, false, 1.0f},
    {2, false, 1.0f}
}};
// 用户 PID 图片参数原样用于 1kHz 速度环/500Hz 位置环/100Hz 轨迹更新。
// 8000 个 C610 协议单位 = 8A 转矩电流指令，并非电源输入电流。
inline device::M2006Position::Config m2006_control[2]{
    {.velocity_pid={9.0f,0.1f,0.0f,8000.0f}, .position_pd={5.0f,4000.0f,400.0f},
     .profile={1800.0f,1200.0f,3000.0f}, .finish_deg=0.5f, .stationary_rpm=0.1f},
    {.velocity_pid={9.0f,0.1f,0.0f,8000.0f}, .position_pd={5.0f,4000.0f,400.0f},
     .profile={1800.0f,1200.0f,3000.0f}, .finish_deg=0.5f, .stationary_rpm=0.1f}
};
} // 命名空间 config
