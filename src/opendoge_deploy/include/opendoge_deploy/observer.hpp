#pragma once

#include <array>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

/// Compute adaptive gait phase matching UniLab training.
/// cmd_speed = norm([vx, vy, vyaw]); freq ∈ [1.2, 2.5] Hz; phase wraps at 1.0.
/// dt is the time step between phase advances (1.0 / inference_hz).
double advancePhase(const OperatorCommand & command, double phase, double dt);

/// Build 49-dim single-frame actor observation matching UniLab deployment spec.
///   gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
///   + last_action(12) + commands(3) + feet_phase(4)
std::array<double, kObsDim> buildObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu,
  double phase);

}  // namespace opendoge
