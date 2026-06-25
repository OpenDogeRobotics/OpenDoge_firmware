#include "opendoge_deploy/controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace opendoge
{

double rateLimit(double desired, double previous, double max_step)
{
  return previous + std::clamp(desired - previous, -max_step, max_step);
}

bool allFeedbackReceived(const std::array<MotorState, kNumJoints> & states)
{
  return std::all_of(states.begin(), states.end(), [](const auto & state) {
    return state.received;
  });
}

void sendDampingBurst(
  El05SocketCan & can,
  const std::array<JointMap, kNumJoints> & joints,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calibration,
  double safe_kd)
{
  for (int repeat = 0; repeat < 20; ++repeat) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double clamped_kd = std::min(safe_kd, calibration[i].max_kd);
      MotorCommand damp{states[i].position, 0.0, 0.0, 0.0, clamped_kd};
      can.sendMotion(joints[i], damp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void updateStateMachine(
  RuntimeState & runtime_state,
  bool & rl_fallback_active,
  OperatorCommand & command,
  const Options & opt,
  const std::array<MotorState, kNumJoints> & states,
  const DeployConfig & config,
  const SafetyConfig & safety,
  const ImuSample & imu,
  double t,
  double & pc_startup_start_s,
  double & feedback_wait_start_s,
  std::string & fault_reason)
{
  // ── Mode priority: RL beats position control ──
  if (command.rl_inference && command.position_control) {
    command.position_control = false;
  }

  // ── WaitFeedback → Ready ──
  if (runtime_state == RuntimeState::WaitFeedback) {
    if (allFeedbackReceived(states)) {
      runtime_state = RuntimeState::Ready;
      feedback_wait_start_s = 0.0;
    } else if (safety.feedback_wait_timeout_s > 0.0) {
      if (feedback_wait_start_s == 0.0) {
        feedback_wait_start_s = t;
      } else if (t - feedback_wait_start_s > safety.feedback_wait_timeout_s) {
        fault_reason = "feedback wait timeout after " + std::to_string(t - feedback_wait_start_s) + "s";
        runtime_state = RuntimeState::DampingFault;
        return;
      }
    }
  }

  // ── Ready ↔ LowGainTest ↔ Active transitions ──
  if (runtime_state == RuntimeState::Ready && command.low_gain_mode) {
    runtime_state = RuntimeState::LowGainTest;
  }
  if (runtime_state == RuntimeState::LowGainTest && !command.low_gain_mode) {
    runtime_state = RuntimeState::Ready;
  }

  if (runtime_state == RuntimeState::Ready && command.active && (imu.valid || opt.allow_missing_imu)) {
    if (command.rl_inference) {
      runtime_state = RuntimeState::ActiveRL;
    } else {
      runtime_state = RuntimeState::EnteringPosition;
      pc_startup_start_s = t;
    }
  }

  // ── EnteringPosition: verify joint positions + check ramp completion ──
  if (runtime_state == RuntimeState::EnteringPosition) {
    if (!command.active) {
      runtime_state = RuntimeState::Ready;
    } else if (command.rl_inference) {
      runtime_state = RuntimeState::ActiveRL;
    } else {
      bool positions_valid = true;
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        const double pos = logicalPosition(states[i].position, config.joints[i]);
        if (std::abs(pos - defaultJointPosition()[i]) > config.pc_startup_max_deviation) {
          positions_valid = false;
          break;
        }
      }
      if (!positions_valid) {
        fault_reason = "position control startup: joint deviation exceeds limit";
        runtime_state = RuntimeState::DampingFault;
      } else if (t - pc_startup_start_s >= config.pc_startup_ramp_s) {
        runtime_state = RuntimeState::ActivePC;
      }
    }
  }

  // ── ActivePC / ActiveRL → Ready ──
  if ((runtime_state == RuntimeState::ActivePC || runtime_state == RuntimeState::ActiveRL) && !command.active) {
    runtime_state = RuntimeState::Ready;
    rl_fallback_active = false;
  }

  // ── ActivePC ↔ ActiveRL switching ──
  if (runtime_state == RuntimeState::ActivePC && command.rl_inference) {
    runtime_state = RuntimeState::ActiveRL;
    rl_fallback_active = false;
  }
  if (runtime_state == RuntimeState::ActiveRL && !command.rl_inference && command.active) {
    runtime_state = RuntimeState::ActivePC;
    rl_fallback_active = false;
  }
}

void computeMotorCommands(
  std::array<MotorCommand, kNumJoints> & commands,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calib,
  const DeployConfig & config,
  RuntimeState runtime_state,
  const std::array<double, kNumJoints> & limited_target,
  const std::array<double, kNumJoints> & default_pos,
  double t,
  double pc_startup_start_s)
{
  const bool is_active = runtime_state == RuntimeState::ActivePC
                      || runtime_state == RuntimeState::ActiveRL;
  const bool is_ramping = runtime_state == RuntimeState::EnteringPosition;
  const bool low_gain = runtime_state == RuntimeState::LowGainTest;

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint_cfg = calib[i];
    if (!is_active && !is_ramping && !low_gain) {
      // 阻尼模式：kp=0, kd=safe_kd (clamped to per-joint max)
      commands[i] = {states[i].position, 0.0, 0.0, 0.0, std::min(config.safe_kd, joint_cfg.max_kd)};
    } else if (is_ramping) {
      // 斜坡：kp/kd 从阻尼值平滑过渡到满 PD 值
      const double ramp_elapsed = t - pc_startup_start_s;
      const double ramp_frac = std::min(ramp_elapsed / config.pc_startup_ramp_s, 1.0);
      const double ramp_kp = ramp_frac * std::min(config.kp, joint_cfg.max_kp);
      const double ramp_kd = config.safe_kd + ramp_frac * (std::min(config.kd, joint_cfg.max_kd) - config.safe_kd);
      commands[i] = {
        motorPosition(limited_target[i], joint_cfg),
        0.0,
        0.0,
        ramp_kp,
        ramp_kd};
    } else {
      double effective_kp = config.kp;
      double effective_kd = config.kd;
      double target = limited_target[i];
      if (low_gain) {
        // Reduced gains + hold default standing pose for safety
        effective_kp = std::min(config.kp * 0.3, joint_cfg.max_kp);
        effective_kd = std::min(config.kd * 0.3, joint_cfg.max_kd);
        target = motorPosition(default_pos[i], joint_cfg);
      }
      commands[i] = {
        target,
        0.0,
        0.0,
        std::min(effective_kp, joint_cfg.max_kp),
        std::min(effective_kd, joint_cfg.max_kd)};
    }
  }
}

void updateTargets(
  std::array<double, kNumJoints> & logical_target,
  std::array<double, kNumJoints> & limited_target,
  std::array<double, kNumJoints> & last_action,
  const std::array<double, kNumJoints> & action,
  const std::array<double, kNumJoints> & default_pos,
  const std::array<JointCalibration, kNumJoints> & calib,
  const DeployConfig & config,
  RuntimeState runtime_state)
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & joint_cfg = calib[i];
    last_action[i] = std::clamp(action[i], -1.0, 1.0);
    if (runtime_state == RuntimeState::LowGainTest) {
      logical_target[i] = default_pos[i];
    } else {
      logical_target[i] = default_pos[i] + last_action[i] * config.action_scale;
    }
    logical_target[i] = std::clamp(logical_target[i], joint_cfg.lower, joint_cfg.upper);
    // RL mode: skip rate_limit (policy trained with instantaneous target changes)
    if (runtime_state == RuntimeState::ActiveRL) {
      limited_target[i] = logical_target[i];
    } else {
      limited_target[i] = rateLimit(logical_target[i], limited_target[i], joint_cfg.max_position_step);
    }
  }
}

}  // namespace opendoge
