#pragma once

#include "device/dm_omni_chassis.hpp"
#include "device/dm_position_test.hpp"
#include "device/m2006_position.hpp"
#include <array>
#include <cstdint>

// Project parameter source. Values are copied when each permanent device is
// constructed; editing these variables afterwards is NOT an online tuning API.
// No HAL handles, object ownership, tasks or hardware initialization belong here.
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
    float reduction_rate{1.0f}; // External only; existing driver handles internal normalization.
};

// Source: approved DM parameter images; mapping ranges, not mechanical travel.
inline constexpr std::array<DMMotorParameters, 6> dm_motor_parameters{{
    {0x0a, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // Right-front linear.
    {0x0b, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // Middle linear.
    {0x0c, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // Rear linear.
    {0x0d, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}, // Left-front joint.
    {0x0e, motors::DMMotor::Type::S3519,     12.5f, 200.0f, 10.0f}, // Left-front linear.
    {0x0f, motors::DMMotor::Type::J4340_2EC, 12.5f,  10.0f, 28.0f}  // Right-front joint.
}};

// Existing low-gain MIT starting values, not loaded tuning or MCU PID gains.
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
}; // No linear software travel interval; mm conversion remains in the device API.

// FR/FL/RL/RR directions, copied at construction; initially positive by user choice.
inline device::DMOmniChassis::Config chassis{};

struct M2006MotorParameters
{
    std::uint8_t id;
    bool reverse;
    float reduction_rate; // External only; DJI handles the internal 36:1.
};
inline constexpr std::array<M2006MotorParameters, 2> m2006_motor_parameters{{
    {1, false, 1.0f},
    {2, false, 1.0f}
}};
// User PID image, used verbatim at 1kHz velocity/500Hz position/100Hz profile.
// 8000 C610 protocol units = 8A torque-current command, NOT supply input current.
inline device::M2006Position::Config m2006_control[2]{
    {.velocity_pid={9.0f,0.1f,0.0f,8000.0f}, .position_pd={5.0f,4000.0f,400.0f},
     .profile={1800.0f,1200.0f,3000.0f}, .finish_deg=0.5f, .stationary_rpm=0.1f},
    {.velocity_pid={9.0f,0.1f,0.0f,8000.0f}, .position_pd={5.0f,4000.0f,400.0f},
     .profile={1800.0f,1200.0f,3000.0f}, .finish_deg=0.5f, .stationary_rpm=0.1f}
};
} // namespace config
