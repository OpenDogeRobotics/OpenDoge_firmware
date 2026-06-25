#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace opendoge
{

constexpr std::size_t kNumJoints = 12;
// Single-frame actor observation (deployable, no privileged info):
//   gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
//   + last_action(12) + commands(3) + feet_phase(4) = 49
// Critic has privileged linvel(3) → 52, but actor does not depend on it.
constexpr std::size_t kObsDim = 49;

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

struct CanStats
{
  std::uint64_t frames_sent{0};
  std::uint64_t frames_received{0};
  std::uint64_t read_errors{0};
  std::uint64_t write_errors{0};
};

/// Per-joint safety state for sustained-condition monitoring.
/// Timers reset when condition clears; a fault is raised only when
/// the condition persists beyond the configured timeout.
struct JointSafetyState
{
  double torque_exceeded_since_s{0.0};
  double tracking_error_since_s{0.0};
};

enum class RuntimeState
{
  WaitFeedback,
  Ready,
  EnteringPosition,
  ActivePC,
  ActiveRL,
  LowGainTest,
  DampingFault,
};

/// Human-readable name for RuntimeState enum values (used in status output).
const char * stateName(RuntimeState state);

struct JointCalibration
{
  double direction{1.0};
  double offset{0.0};
  double reduction{1.0};  // motor-to-joint gear ratio (>1: motor spins faster)
  double lower{-12.57};
  double upper{12.57};
  double max_position_step{0.015};
  double max_velocity{20.0};
  double max_torque{3.0};
  double max_kp{50.0};
  double max_kd{5.0};  // Matches EL05 spec: kd ∈ [0, 5]
};

struct DeployConfig
{
  double inference_hz{100.0};
  double target_hz{200.0};
  double control_hz{1000.0};
  double kp{20.0};
  double kd{0.3};
  double safe_kd{2.0};
  double action_scale{0.50};
  double state_timeout_s{0.02};
  double over_temperature_c{80.0};
  double fault_poll_hz{10.0};
  double pc_startup_ramp_s{2.0};
  double pc_startup_max_deviation{0.25};
  // Safety monitoring thresholds
  double torque_threshold{3.0};
  double torque_timeout_s{0.5};
  double tracking_error_threshold{0.5};
  double tracking_error_timeout_s{0.3};
  double command_timeout_s{0.5};
  double fall_gravity_z_threshold{0.3};
  double fall_timeout_s{0.3};
  // Command smoothing (0 = disabled)
  double command_smoothing_alpha{0.0};
  // WaitFeedback overall timeout (0 = no timeout)
  double feedback_wait_timeout_s{5.0};
  // Early temperature warning threshold (C)
  double temp_warn_c{65.0};
  // IMU consecutive invalid reads before dropping to Ready
  int imu_debounce_count{10};
  std::array<JointCalibration, kNumJoints> joints{};
};

struct OperatorCommand
{
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  bool active{false};
  bool estop{false};
  bool position_control{false};
  bool rl_inference{false};
  bool clear_fault{false};
  bool low_gain_mode{false};
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

// Matches UniLab scene_flat.xml keyframe qpos (joint portion):
//   FL: 0, 0.5, -1.3  FR: 0, 0.5, -1.3
//   RL: 0, 0.7, -1.3  RR: 0, 0.7, -1.3
// Front/rear asymmetry is intentional — it centres the CoM between legs
// and is what the policy was trained against.
inline std::array<double, kNumJoints> defaultJointPosition()
{
  return {0.0, 0.5, -1.3, 0.0, 0.5, -1.3, 0.0, 0.7, -1.3, 0.0, 0.7, -1.3};
}

inline std::array<JointCalibration, kNumJoints> defaultJointCalibration()
{
  // Physically reasonable defaults matching OpenDoge URDF constraints.
  // Config file overrides these — these are safety fallbacks only.
  return {{
    {1.0, 0.0, -0.785, 0.26}, {1.0, 0.0, -0.785, 1.134}, {1.0, 0.0, -2.68, -1.04},
    {1.0, 0.0, -0.26, 0.785},  {1.0, 0.0, -0.785, 1.134}, {1.0, 0.0, -2.68, -1.04},
    {1.0, 0.0, -0.785, 0.26}, {1.0, 0.0, -0.785, 1.134}, {1.0, 0.0, -2.68, -1.04},
    {1.0, 0.0, -0.26, 0.785},  {1.0, 0.0, -0.785, 1.134}, {1.0, 0.0, -2.68, -1.04},
  }};
}

inline double logicalPosition(double motor_position, const JointCalibration & calibration)
{
  return calibration.direction * (motor_position / calibration.reduction - calibration.offset);
}

inline double logicalVelocity(double motor_velocity, const JointCalibration & calibration)
{
  return calibration.direction * motor_velocity / calibration.reduction;
}

inline double motorPosition(double logical_position, const JointCalibration & calibration)
{
  return (calibration.offset + calibration.direction * logical_position) * calibration.reduction;
}

}  // namespace opendoge
