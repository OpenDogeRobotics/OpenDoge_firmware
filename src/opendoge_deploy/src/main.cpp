#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <sys/stat.h>

#include "opendoge_deploy/cli.hpp"
#include "opendoge_deploy/controller.hpp"
#include "opendoge_deploy/el05_socketcan.hpp"
#include "opendoge_deploy/observer.hpp"
#include "opendoge_deploy/policy.hpp"
#include "opendoge_deploy/runtime_io.hpp"
#include "opendoge_deploy/safety.hpp"
#include "opendoge_deploy/status.hpp"
#include "opendoge_deploy/types.hpp"

namespace
{
std::atomic_bool g_stop{false};

void signalHandler(int)
{
  g_stop.store(true);
}

double nowSeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path);
  return static_cast<bool>(file);
}
}  // namespace

int main(int argc, char ** argv)
{
  opendoge::Options opt;
  if (!opendoge::parseArgs(argc, argv, opt)) {
    return 1;
  }

  if (opt.config_path.empty()) {
    const std::array<std::string, 3> default_config_paths{
      "configs/opendoge_deploy.conf",
      "src/opendoge_deploy/configs/opendoge_deploy.conf",
      "install/opendoge_deploy/share/opendoge_deploy/configs/opendoge_deploy.conf"};
    for (const auto & path : default_config_paths) {
      if (fileExists(path)) {
        opt.config_path = path;
        break;
      }
    }
  }
  if (opt.dry_run) {
    opt.allow_missing_imu = true;
  }
  opendoge::applyRuntimeTuning(opt);

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto joints = opendoge::defaultJointMap();
  const auto default_pos = opendoge::defaultJointPosition();
  opendoge::DeployConfig config;
  std::string error;
  if (!opendoge::loadDeployConfig(opt.config_path, joints, config, error)) {
    std::cerr << "Config load failed: " << error << "\n";
    return 1;
  }

  std::array<opendoge::MotorState, opendoge::kNumJoints> states{};
  std::array<opendoge::MotorCommand, opendoge::kNumJoints> commands{};
  std::array<double, opendoge::kNumJoints> action{};
  std::array<double, opendoge::kNumJoints> last_action{};
  std::array<double, opendoge::kObsDim> obs{};
  std::array<double, opendoge::kNumJoints> logical_target = default_pos;
  std::array<double, opendoge::kNumJoints> limited_target = default_pos;
  double phase = 0.0;

  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    states[i].position = opendoge::motorPosition(default_pos[i], config.joints[i]);
    commands[i] = {states[i].position, 0.0, 0.0, 0.0, config.safe_kd};
  }

  auto policy = opendoge::makePolicy(opt.policy_backend);
  if (!policy) {
    std::cerr << "Unknown policy backend: " << opt.policy_backend << "\n";
    return 1;
  }
  if (!policy->load(opt.policy_path, error)) {
    std::cerr << "Policy load failed: " << error << "\n";
    return 1;
  }

  opendoge::El05SocketCan can;
  if (!opt.dry_run) {
    if (!can.open(joints)) {
      std::cerr << "CAN open failed: " << can.lastError() << "\n";
      return 1;
    }
    if (opt.enable) {
      for (const auto & joint : joints) {
        if (opt.clear_fault) {
          if (!can.sendStop(joint, true)) {
            std::cerr << "Clear fault failed: " << can.lastError() << "\n";
            opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
            can.close();
            return 1;
          }
        }
        if (!can.sendMotionMode(joint) || !can.sendEnable(joint)) {
          std::cerr << "Motor startup failed: " << can.lastError() << "\n";
          opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
          can.close();
          return 1;
        }
      }
    }
  }

  std::cout << "OpenDoge deploy running: "
            << (opt.dry_run ? "dry-run" : "real CAN")
            << ", policy=" << opt.policy_backend
            << ", control=" << config.control_hz << "Hz"
            << ", config=" << (opt.config_path.empty() ? "defaults" : opt.config_path)
            << "\n";

  const double start_s = nowSeconds();
  double next_control_s = start_s;
  double next_infer_s = start_s;
  double next_target_s = start_s;
  double next_fault_poll_s = start_s;
  double next_status_s = start_s + 1.0;
  opendoge::RuntimeState runtime_state = opt.dry_run ? opendoge::RuntimeState::Ready : opendoge::RuntimeState::WaitFeedback;
  opendoge::OperatorCommand command = opt.static_command;
  opendoge::ImuSample imu;
  imu.valid = opt.allow_missing_imu;
  std::string fault_reason;
  double next_input_s = start_s;
  opendoge::LoopStats loop_stats;
  double pc_startup_start_s{0.0};
  bool rl_fallback_active{false};
  double feedback_wait_start_s{0.0};
  int imu_invalid_count{0};
  std::array<opendoge::JointSafetyState, opendoge::kNumJoints> safety_state{};
  std::uint64_t last_can_sent = 0;
  std::uint64_t last_can_received = 0;

  while (!g_stop.load()) {
    const double t = nowSeconds();
    if (opt.duration_s > 0.0 && t - start_s >= opt.duration_s) {
      break;
    }

    // ── CAN drain ──
    if (!opt.dry_run) {
      can.drain(states, t);
      if (!can.ok()) {
        fault_reason = can.lastError();
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
    }

    // ── Fault polling ──
    if (t >= next_fault_poll_s && !opt.dry_run) {
      for (const auto & joint : joints) {
        can.sendReadFaultStatus(joint);
      }
      next_fault_poll_s = t + 1.0 / std::max(config.fault_poll_hz, 1.0);
    }

    // ── Input polling ──
    if (t >= next_input_s) {
      if (!opendoge::readCommandFile(opt.command_file, command, error)) {
        fault_reason = error;
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      command.active = command.active || opt.start_active;
      if (command.estop) {
        fault_reason = "operator estop";
        runtime_state = opendoge::RuntimeState::DampingFault;
      }

      // Command timeout: stale command file while active → zero commands
      if (!opt.dry_run && command.active && !opt.command_file.empty()) {
        struct stat cmd_stat {};
        if (::stat(opt.command_file.c_str(), &cmd_stat) == 0) {
          const double file_age_s = t - std::max(
            static_cast<double>(cmd_stat.st_mtime),
            static_cast<double>(cmd_stat.st_ctime));
          if (file_age_s > config.command_timeout_s) {
            std::cerr << "Warning: command file stale for "
                      << file_age_s << "s, zeroing commands\n";
            command.vx = 0.0;
            command.vy = 0.0;
            command.yaw_rate = 0.0;
            command.active = false;
          }
        }
      }

      // Command smoothing (EMA low-pass)
      if (config.command_smoothing_alpha > 0.0) {
        static opendoge::OperatorCommand smooth_cmd = command;
        const double a = config.command_smoothing_alpha;
        smooth_cmd.vx = a * command.vx + (1.0 - a) * smooth_cmd.vx;
        smooth_cmd.vy = a * command.vy + (1.0 - a) * smooth_cmd.vy;
        smooth_cmd.yaw_rate = a * command.yaw_rate + (1.0 - a) * smooth_cmd.yaw_rate;
        command.vx = smooth_cmd.vx;
        command.vy = smooth_cmd.vy;
        command.yaw_rate = smooth_cmd.yaw_rate;
        smooth_cmd.active = command.active;
        smooth_cmd.estop = command.estop;
        smooth_cmd.position_control = command.position_control;
        smooth_cmd.rl_inference = command.rl_inference;
        smooth_cmd.clear_fault = command.clear_fault;
        smooth_cmd.low_gain_mode = command.low_gain_mode;
      }

      if (!opendoge::readImuFile(opt.imu_file, imu, t, error)) {
        fault_reason = error;
        runtime_state = opendoge::RuntimeState::DampingFault;
      }
      if (!opt.allow_missing_imu && !imu.valid) {
        ++imu_invalid_count;
      } else {
        imu_invalid_count = 0;
      }
      if (!opt.allow_missing_imu && imu_invalid_count > config.imu_debounce_count) {
        runtime_state = opendoge::RuntimeState::Ready;
      }
      next_input_s = t + 0.005;
    }

    // ── Safety checks (non-WaitFeedback states) ──
    if (!opt.dry_run && runtime_state != opendoge::RuntimeState::WaitFeedback) {
      std::string safety_reason;
      const bool fault_now = opendoge::safetyFault(states, joints, config.joints, config, runtime_state,
            limited_target, imu, safety_state, t, safety_reason);
      if (fault_now) {
        fault_reason = safety_reason;
        runtime_state = opendoge::RuntimeState::DampingFault;
      } else if (runtime_state == opendoge::RuntimeState::DampingFault && command.clear_fault) {
        bool can_ok = !opt.dry_run ? can.ok() : true;
        if (can_ok) {
          for (const auto & joint : joints) {
            can.sendStop(joint, true);
            can.sendMotionMode(joint);
            can.sendEnable(joint);
          }
        }
        std::cout << "Fault cleared, transitioning to WaitFeedback"
                  << (can_ok ? "" : " (CAN down, waiting for link)") << "\n";
        fault_reason.clear();
        runtime_state = opendoge::RuntimeState::WaitFeedback;
        for (auto & ss : safety_state) {
          ss = opendoge::JointSafetyState{};
        }
        for (auto & st : states) {
          st.received = false;
        }
        command.clear_fault = false;
      }
    }

    // ── State transitions (non-fault paths) ──
    if (runtime_state != opendoge::RuntimeState::DampingFault) {
      opendoge::updateStateMachine(
        runtime_state, rl_fallback_active, command, opt,
        states, config, imu, t,
        pc_startup_start_s, feedback_wait_start_s, fault_reason);
    }

    // ── Inference block (100 Hz) ──
    if (t >= next_infer_s) {
      ++loop_stats.inference_ticks;
      if ((runtime_state == opendoge::RuntimeState::ActiveRL || runtime_state == opendoge::RuntimeState::EnteringPosition
           || runtime_state == opendoge::RuntimeState::ActivePC) && command.active) {
        phase = opendoge::advancePhase(command, phase, 1.0 / config.inference_hz);
        obs = opendoge::buildObservation(states, config.joints, default_pos, last_action, command, imu, phase);
        if (!policy->infer(obs, action, error)) {
          if (runtime_state == opendoge::RuntimeState::ActiveRL) {
            runtime_state = opendoge::RuntimeState::ActivePC;
            rl_fallback_active = true;
            action.fill(0.0);
          } else {
            fault_reason = "policy infer failed: " + error;
            runtime_state = opendoge::RuntimeState::DampingFault;
          }
        }
      } else {
        action.fill(0.0);
      }
      next_infer_s = t + 1.0 / config.inference_hz;
    }

    // ── Target update block (200 Hz) ──
    if (t >= next_target_s) {
      ++loop_stats.target_ticks;
      opendoge::updateTargets(
        logical_target, limited_target, last_action, action,
        default_pos, config.joints, config, runtime_state);
      next_target_s = t + 1.0 / config.target_hz;
    }

    // ── Control block (1000 Hz) ──
    if (t >= next_control_s) {
      ++loop_stats.control_ticks;
      const double control_late_s = t - next_control_s;
      loop_stats.max_control_late_s = std::max(loop_stats.max_control_late_s, control_late_s);
      if (control_late_s > 0.0005) {
        ++loop_stats.missed_control_deadlines;
      }
      const bool can_ready = !opt.dry_run && can.ok();
      opendoge::computeMotorCommands(
        commands, states, config.joints, config,
        runtime_state, limited_target, default_pos, t, pc_startup_start_s);
      if (can_ready) {
        for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
          if (!can.sendMotion(joints[i], commands[i])) {
            fault_reason = can.lastError();
            runtime_state = opendoge::RuntimeState::DampingFault;
            break;
          }
        }
      }
      next_control_s = t + 1.0 / config.control_hz;
    }

    // ── Status output (1 Hz) ──
    if (t >= next_status_s) {
      opendoge::emitStatus(
        runtime_state, command, opt.status_file,
        loop_stats, last_can_sent, last_can_received,
        can.stats(), fault_reason, rl_fallback_active,
        states, config.joints, joints, imu,
        pc_startup_start_s, config.pc_startup_ramp_s, t);
      next_status_s = t + 1.0;
    }

    const double next_deadline_s = std::min(
      std::min(next_control_s, next_infer_s),
      std::min(std::min(next_target_s, next_fault_poll_s), std::min(next_status_s, next_input_s)));
    opendoge::waitUntilNextDeadline(next_deadline_s);
  }

  if (!opt.dry_run) {
    opendoge::sendDampingBurst(can, joints, states, config.joints, config.safe_kd);
    can.close();
  }

  std::cout << "OpenDoge deploy stopped";
  if (runtime_state == opendoge::RuntimeState::DampingFault) {
    std::cout << " with fault: " << fault_reason;
  }
  std::cout << "\n";
  return runtime_state == opendoge::RuntimeState::DampingFault ? 2 : 0;
}
