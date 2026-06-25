#pragma once

#include <array>
#include <string>

#include "opendoge_deploy/types.hpp"

namespace opendoge
{

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

const char * stateName(RuntimeState state);

/// Check all safety conditions. Returns true (with reason) if a fault is detected.
bool safetyFault(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointMap, kNumJoints> & joints,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const SafetyConfig & safety,
  RuntimeState runtime_state,
  const std::array<double, kNumJoints> & logical_target,
  const ImuSample & imu,
  std::array<JointSafetyState, kNumJoints> & safety_state,
  double now_s,
  std::string & reason);

}  // namespace opendoge
