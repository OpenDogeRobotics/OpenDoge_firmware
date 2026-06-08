#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

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

}  // namespace opendoge
