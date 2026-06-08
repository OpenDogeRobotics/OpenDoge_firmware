#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "opendoge_deploy/el05_socketcan.hpp"
#include "opendoge_deploy/policy.hpp"
#include "opendoge_deploy/runtime_io.hpp"
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

enum class RuntimeState
{
  WaitFeedback,
  Ready,
  Active,
  DampingFault,
};

const char * stateName(RuntimeState state)
{
  switch (state) {
    case RuntimeState::WaitFeedback:
      return "wait_feedback";
    case RuntimeState::Ready:
      return "ready";
    case RuntimeState::Active:
      return "active";
    case RuntimeState::DampingFault:
      return "damping_fault";
  }
  return "unknown";
}

struct Options
{
  std::string policy_backend{"none"};
  std::string policy_path;
  std::string config_path;
  std::string command_file;
  std::string imu_file;
  bool dry_run{true};
  bool enable{false};
  bool clear_fault{false};
  bool start_active{false};
  bool allow_missing_imu{false};
  double duration_s{0.0};
  opendoge::OperatorCommand static_command;
};

void printUsage()
{
  std::cout
    << "Usage: opendoge_deploy [options]\n"
    << "  --policy-backend none|linear_csv|onnx   default: none\n"
    << "  --policy-path PATH                      ONNX or linear CSV path\n"
    << "  --config PATH                           deploy key=value config\n"
    << "  --command-file PATH                     vx/vy/yaw_rate/active/estop input\n"
    << "  --imu-file PATH                         wx/wy/wz/gx/gy/gz input\n"
    << "  --cmd VX VY YAW                         static command\n"
    << "  --real                                  open can0..can3 and send frames\n"
    << "  --enable                                set motion mode and enable motors on startup\n"
    << "  --clear-fault                           send stop(clear_fault=1) before enable\n"
    << "  --start-active                          enter active after readiness checks\n"
    << "  --allow-missing-imu                     permit active with default IMU sample\n"
    << "  --duration-sec SEC                      stop after SEC, 0 means until Ctrl+C\n";
}

bool parseArgs(int argc, char ** argv, Options & opt)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto need_value = [&](const std::string & name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + name);
      }
      return argv[++i];
    };
    try {
      if (arg == "--help" || arg == "-h") {
        printUsage();
        return false;
      } else if (arg == "--policy-backend") {
        opt.policy_backend = need_value(arg);
      } else if (arg == "--policy-path") {
        opt.policy_path = need_value(arg);
      } else if (arg == "--config") {
        opt.config_path = need_value(arg);
      } else if (arg == "--command-file") {
        opt.command_file = need_value(arg);
      } else if (arg == "--imu-file") {
        opt.imu_file = need_value(arg);
      } else if (arg == "--cmd") {
        opt.static_command.vx = std::stod(need_value(arg));
        opt.static_command.vy = std::stod(need_value(arg));
        opt.static_command.yaw_rate = std::stod(need_value(arg));
      } else if (arg == "--real") {
        opt.dry_run = false;
      } else if (arg == "--enable") {
        opt.enable = true;
      } else if (arg == "--clear-fault") {
        opt.clear_fault = true;
      } else if (arg == "--start-active") {
        opt.start_active = true;
        opt.static_command.active = true;
      } else if (arg == "--allow-missing-imu") {
        opt.allow_missing_imu = true;
      } else if (arg == "--duration-sec") {
        opt.duration_s = std::stod(need_value(arg));
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    } catch (const std::exception & exc) {
      std::cerr << exc.what() << "\n";
      return false;
    }
  }
  return true;
}

std::array<double, opendoge::kObsDim> buildObservation(
  const std::array<opendoge::MotorState, opendoge::kNumJoints> & states,
  const std::array<opendoge::JointCalibration, opendoge::kNumJoints> & calibration,
  const std::array<double, opendoge::kNumJoints> & default_pos,
  const std::array<double, opendoge::kNumJoints> & last_action,
  const opendoge::OperatorCommand & command,
  const opendoge::ImuSample & imu,
  const std::array<double, opendoge::kObsDim> & previous)
{
  std::array<double, opendoge::kObsDim> obs{};

  std::array<double, opendoge::kOneStepObs> one{};
  one[0] = command.vx * 2.0;
  one[1] = command.vy * 2.0;
  one[2] = command.yaw_rate * 0.25;
  for (std::size_t i = 0; i < 3; ++i) {
    one[3 + i] = imu.angular_velocity[i] * 0.25;
    one[6 + i] = imu.projected_gravity[i];
  }
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    const auto pos = opendoge::logicalPosition(states[i].position, calibration[i]);
    const auto vel = opendoge::logicalVelocity(states[i].velocity, calibration[i]);
    one[9 + i] = pos - default_pos[i];
    one[9 + opendoge::kNumJoints + i] = vel * 0.05;
    one[9 + opendoge::kNumJoints * 2 + i] = last_action[i];
  }

  std::copy(one.begin(), one.end(), obs.begin());
  std::copy(
    previous.begin(),
    previous.begin() + static_cast<std::ptrdiff_t>(opendoge::kObsDim - opendoge::kOneStepObs),
    obs.begin() + static_cast<std::ptrdiff_t>(opendoge::kOneStepObs));
  return obs;
}

bool allFeedbackReceived(const std::array<opendoge::MotorState, opendoge::kNumJoints> & states)
{
  return std::all_of(states.begin(), states.end(), [](const auto & state) {
    return state.received;
  });
}

bool safetyFault(
  const std::array<opendoge::MotorState, opendoge::kNumJoints> & states,
  const std::array<opendoge::JointMap, opendoge::kNumJoints> & joints,
  const opendoge::SafetyConfig & safety,
  double now_s,
  std::string & reason)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (!states[i].received) {
      reason = joints[i].name + ": missing feedback";
      return true;
    }
    if (now_s - states[i].last_feedback_s > safety.state_timeout_s) {
      reason = joints[i].name + ": feedback timeout";
      return true;
    }
    if (states[i].fault != 0) {
      reason = joints[i].name + ": status fault bits=0x" + std::to_string(states[i].fault);
      return true;
    }
    if (states[i].param_fault != 0) {
      reason = joints[i].name + ": faultSta=0x" + std::to_string(states[i].param_fault);
      return true;
    }
    if (states[i].temperature >= safety.over_temperature_c) {
      reason = joints[i].name + ": over temperature";
      return true;
    }
  }
  return false;
}

double rateLimit(double desired, double previous, double max_step)
{
  return previous + std::clamp(desired - previous, -max_step, max_step);
}

void sendDampingBurst(
  opendoge::El05SocketCan & can,
  const std::array<opendoge::JointMap, opendoge::kNumJoints> & joints,
  const std::array<opendoge::MotorState, opendoge::kNumJoints> & states,
  double safe_kd)
{
  for (int repeat = 0; repeat < 20; ++repeat) {
    for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
      opendoge::MotorCommand damp{states[i].position, 0.0, 0.0, 0.0, safe_kd};
      can.sendMotion(joints[i], damp);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
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
            sendDampingBurst(can, joints, states, config.safe_kd);
            can.close();
            return 1;
          }
        }
        if (!can.sendMotionMode(joint) || !can.sendEnable(joint)) {
          std::cerr << "Motor startup failed: " << can.lastError() << "\n";
          sendDampingBurst(can, joints, states, config.safe_kd);
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
  double next_status_s = start_s;
  RuntimeState runtime_state = opt.dry_run ? RuntimeState::Ready : RuntimeState::WaitFeedback;
  opendoge::SafetyConfig safety;
  safety.safe_kd = config.safe_kd;
  safety.state_timeout_s = config.state_timeout_s;
  safety.over_temperature_c = config.over_temperature_c;
  opendoge::OperatorCommand command = opt.static_command;
  opendoge::ImuSample imu;
  imu.valid = opt.allow_missing_imu;
  bool fault_latched = false;
  std::string fault_reason;
  double next_input_s = start_s;

  while (!g_stop.load()) {
    const double t = nowSeconds();
    if (opt.duration_s > 0.0 && t - start_s >= opt.duration_s) {
      break;
    }

    if (!opt.dry_run) {
      can.drain(states, t);
      if (!can.ok()) {
        fault_latched = true;
        fault_reason = can.lastError();
        runtime_state = RuntimeState::DampingFault;
      }
    }

    if (t >= next_fault_poll_s && !opt.dry_run) {
      for (const auto & joint : joints) {
        can.sendReadFaultStatus(joint);
      }
      next_fault_poll_s = t + 1.0 / std::max(config.fault_poll_hz, 1.0);
    }

    if (t >= next_input_s) {
      if (!opendoge::readCommandFile(opt.command_file, command, error)) {
        fault_latched = true;
        fault_reason = error;
        runtime_state = RuntimeState::DampingFault;
      }
      command.active = command.active || opt.start_active;
      if (command.estop) {
        fault_latched = true;
        fault_reason = "operator estop";
        runtime_state = RuntimeState::DampingFault;
      }

      if (!opendoge::readImuFile(opt.imu_file, imu, t, error)) {
        fault_latched = true;
        fault_reason = error;
        runtime_state = RuntimeState::DampingFault;
      }
      if (!opt.allow_missing_imu && !imu.valid) {
        runtime_state = RuntimeState::Ready;
      }
      next_input_s = t + 0.005;
    }

    if (!fault_latched && !opt.dry_run && runtime_state != RuntimeState::WaitFeedback) {
      std::string safety_reason;
      if (safetyFault(states, joints, safety, t, safety_reason)) {
        fault_latched = true;
        fault_reason = safety_reason;
        runtime_state = RuntimeState::DampingFault;
      }
    }

    if (!fault_latched) {
      if (runtime_state == RuntimeState::WaitFeedback && allFeedbackReceived(states)) {
        runtime_state = RuntimeState::Ready;
      }
      if (runtime_state == RuntimeState::Ready && command.active && (imu.valid || opt.allow_missing_imu)) {
        runtime_state = RuntimeState::Active;
      }
      if (runtime_state == RuntimeState::Active && !command.active) {
        runtime_state = RuntimeState::Ready;
      }
    }

    if (t >= next_infer_s) {
      obs = buildObservation(states, config.joints, default_pos, last_action, command, imu, obs);
      if (!policy->infer(obs, action, error)) {
        fault_latched = true;
        fault_reason = "policy infer failed: " + error;
        runtime_state = RuntimeState::DampingFault;
      }
      next_infer_s = t + 1.0 / config.inference_hz;
    }

    if (t >= next_target_s) {
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        const auto & joint_cfg = config.joints[i];
        last_action[i] = std::clamp(action[i], -1.0, 1.0);
        logical_target[i] = default_pos[i] + last_action[i] * config.action_scale;
        logical_target[i] = std::clamp(logical_target[i], joint_cfg.lower, joint_cfg.upper);
        limited_target[i] = rateLimit(logical_target[i], limited_target[i], joint_cfg.max_position_step);
      }
      next_target_s = t + 1.0 / config.target_hz;
    }

    if (t >= next_control_s) {
      const bool active = runtime_state == RuntimeState::Active;
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        const auto & joint_cfg = config.joints[i];
        if (!active) {
          commands[i] = {states[i].position, 0.0, 0.0, 0.0, config.safe_kd};
        } else {
          commands[i] = {
            opendoge::motorPosition(limited_target[i], joint_cfg),
            0.0,
            0.0,
            std::min(config.kp, joint_cfg.max_kp),
            std::min(config.kd, joint_cfg.max_kd)};
        }
        if (!opt.dry_run && !can.sendMotion(joints[i], commands[i])) {
          fault_latched = true;
          fault_reason = can.lastError();
          runtime_state = RuntimeState::DampingFault;
        }
      }
      next_control_s = t + 1.0 / config.control_hz;
    }

    if (t >= next_status_s) {
      std::cout << "state=" << stateName(runtime_state)
                << " active_cmd=" << (command.active ? 1 : 0)
                << " imu=" << (imu.valid ? 1 : 0);
      if (fault_latched) {
        std::cout << " fault=\"" << fault_reason << "\"";
      }
      std::cout << "\n";
      next_status_s = t + 1.0;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (!opt.dry_run) {
    sendDampingBurst(can, joints, states, config.safe_kd);
    can.close();
  }

  std::cout << "OpenDoge deploy stopped";
  if (fault_latched) {
    std::cout << " with fault: " << fault_reason;
  }
  std::cout << "\n";
  return fault_latched ? 2 : 0;
}
