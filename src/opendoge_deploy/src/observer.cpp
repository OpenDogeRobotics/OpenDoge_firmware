#include "opendoge_deploy/observer.hpp"

#include <algorithm>
#include <cmath>

namespace opendoge
{

double advancePhase(const OperatorCommand & command, double phase, double dt)
{
  const double cmd_speed = std::sqrt(
    command.vx * command.vx + command.vy * command.vy + command.yaw_rate * command.yaw_rate);
  const double freq = std::clamp(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5);
  return std::fmod(phase + dt * freq, 1.0);
}

std::array<double, kObsDim> buildObservation(
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<double, kNumJoints> & last_action,
  const OperatorCommand & command,
  const ImuSample & imu,
  double phase)
{
  std::array<double, kObsDim> obs{};
  std::size_t offset = 0;

  // 1. gyro (angular velocity) — 3 dims, no scaling
  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.angular_velocity[i];
  }
  offset += 3;

  // 2. negated gravity (projected_gravity from IMU is already "down" = -upvector) — 3 dims
  for (std::size_t i = 0; i < 3; ++i) {
    obs[offset + i] = imu.projected_gravity[i];
  }
  offset += 3;

  // 3. dof_pos - default_pos — 12 dims
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto pos = logicalPosition(states[i].position, calibration[i]);
    obs[offset + i] = pos - default_pos[i];
  }
  offset += kNumJoints;

  // 4. dof_vel — 12 dims
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = logicalVelocity(states[i].velocity, calibration[i]);
  }
  offset += kNumJoints;

  // 5. last_action — 12 dims
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    obs[offset + i] = last_action[i];
  }
  offset += kNumJoints;

  // 6. commands (raw, no scaling) — 3 dims
  obs[offset + 0] = command.vx;
  obs[offset + 1] = command.vy;
  obs[offset + 2] = command.yaw_rate;
  offset += 3;

  // 7. feet_phase — 4 dims
  //    FL=phase, FR=(phase+0.5)%1, RL=(phase+0.5)%1, RR=phase
  obs[offset + 0] = phase;                       // FL
  obs[offset + 1] = std::fmod(phase + 0.5, 1.0); // FR
  obs[offset + 2] = std::fmod(phase + 0.5, 1.0); // RL
  obs[offset + 3] = phase;                       // RR

  return obs;
}

}  // namespace opendoge
