#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace opendoge
{

constexpr std::size_t kNumJoints = 12;
constexpr std::size_t kOneStepObs = 45;
constexpr std::size_t kFrameStack = 6;
constexpr std::size_t kObsDim = kOneStepObs * kFrameStack;

struct JointMap
{
  std::string name;
  std::string can;
  int motor_id;
};

struct MotorState
{
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double temperature{0.0};
  std::uint16_t fault{0};
  std::uint32_t param_fault{0};
  bool received{false};
  double last_feedback_s{0.0};
};

struct MotorCommand
{
  double q{0.0};
  double dq{0.0};
  double tau{0.0};
  double kp{0.0};
  double kd{0.0};
};

struct SafetyConfig
{
  double state_timeout_s{0.02};
  double over_temperature_c{80.0};
  double safe_kd{2.0};
};

struct JointCalibration
{
  double direction{1.0};
  double offset{0.0};
  double lower{-12.57};
  double upper{12.57};
  double max_position_step{0.015};
  double max_velocity{20.0};
  double max_torque{3.0};
  double max_kp{50.0};
  double max_kd{2.0};
};

struct DeployConfig
{
  double inference_hz{50.0};
  double target_hz{200.0};
  double control_hz{1000.0};
  double kp{12.0};
  double kd{0.5};
  double safe_kd{2.0};
  double action_scale{0.30};
  double state_timeout_s{0.02};
  double over_temperature_c{80.0};
  double fault_poll_hz{10.0};
  std::array<JointCalibration, kNumJoints> joints{};
};

struct OperatorCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool active{false};
  bool estop{false};
};

struct ImuSample
{
  std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
  std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};
  bool valid{false};
  double last_update_s{0.0};
};

inline std::array<JointMap, kNumJoints> defaultJointMap()
{
  return {{
    {"FL_hip_joint", "can0", 1}, {"FL_thigh_joint", "can0", 2}, {"FL_calf_joint", "can0", 3},
    {"FR_hip_joint", "can1", 4}, {"FR_thigh_joint", "can1", 5}, {"FR_calf_joint", "can1", 6},
    {"RL_hip_joint", "can2", 7}, {"RL_thigh_joint", "can2", 8}, {"RL_calf_joint", "can2", 9},
    {"RR_hip_joint", "can3", 10}, {"RR_thigh_joint", "can3", 11}, {"RR_calf_joint", "can3", 12},
  }};
}

inline std::array<double, kNumJoints> defaultJointPosition()
{
  return {0.0, 0.6, -1.5, 0.0, 0.6, -1.5, 0.0, 0.6, -1.5, 0.0, 0.6, -1.5};
}

inline std::array<JointCalibration, kNumJoints> defaultJointCalibration()
{
  return {{
    {1.0, 0.0, -1.57, 1.57}, {1.0, 0.0, -2.0, 2.0}, {1.0, 0.0, -2.5, 0.0},
    {1.0, 0.0, -1.57, 1.57}, {1.0, 0.0, -2.0, 2.0}, {1.0, 0.0, -2.5, 0.0},
    {1.0, 0.0, -1.57, 1.57}, {1.0, 0.0, -2.0, 2.0}, {1.0, 0.0, -2.5, 0.0},
    {1.0, 0.0, -1.57, 1.57}, {1.0, 0.0, -2.0, 2.0}, {1.0, 0.0, -2.5, 0.0},
  }};
}

inline double logicalPosition(double motor_position, const JointCalibration & calibration)
{
  return calibration.direction * (motor_position - calibration.offset);
}

inline double logicalVelocity(double motor_velocity, const JointCalibration & calibration)
{
  return calibration.direction * motor_velocity;
}

inline double motorPosition(double logical_position, const JointCalibration & calibration)
{
  return calibration.offset + calibration.direction * logical_position;
}

}  // namespace opendoge
