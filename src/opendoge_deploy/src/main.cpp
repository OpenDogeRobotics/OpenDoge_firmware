#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "opendoge_deploy/el05_socketcan.hpp"
#include "opendoge_deploy/policy.hpp"
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

struct Options
{
  std::string policy_backend{"none"};
  std::string policy_path;
  bool dry_run{true};
  bool enable{false};
  bool clear_fault{false};
  double duration_s{0.0};
  double inference_hz{50.0};
  double target_hz{200.0};
  double control_hz{1000.0};
  double kp{12.0};
  double kd{0.5};
  double safe_kd{2.0};
  double action_scale{0.30};
};

void printUsage()
{
  std::cout
    << "Usage: opendoge_deploy [options]\n"
    << "  --policy-backend none|linear_csv|onnx   default: none\n"
    << "  --policy-path PATH                      ONNX or linear CSV path\n"
    << "  --real                                  open can0..can3 and send frames\n"
    << "  --enable                                set motion mode and enable motors on startup\n"
    << "  --clear-fault                           send stop(clear_fault=1) before enable\n"
    << "  --duration-sec SEC                      stop after SEC, 0 means until Ctrl+C\n"
    << "  --kp VALUE                              default: 12\n"
    << "  --kd VALUE                              default: 0.5\n"
    << "  --safe-kd VALUE                         default: 2.0\n";
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
      } else if (arg == "--real") {
        opt.dry_run = false;
      } else if (arg == "--enable") {
        opt.enable = true;
      } else if (arg == "--clear-fault") {
        opt.clear_fault = true;
      } else if (arg == "--duration-sec") {
        opt.duration_s = std::stod(need_value(arg));
      } else if (arg == "--kp") {
        opt.kp = std::stod(need_value(arg));
      } else if (arg == "--kd") {
        opt.kd = std::stod(need_value(arg));
      } else if (arg == "--safe-kd") {
        opt.safe_kd = std::stod(need_value(arg));
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
  const std::array<double, opendoge::kNumJoints> & default_pos,
  const std::array<double, opendoge::kNumJoints> & last_action,
  const std::array<double, 3> & command,
  const std::array<double, 3> & angular_velocity,
  const std::array<double, 3> & projected_gravity,
  const std::array<double, opendoge::kObsDim> & previous)
{
  std::array<double, opendoge::kObsDim> obs{};

  std::array<double, opendoge::kOneStepObs> one{};
  one[0] = command[0] * 2.0;
  one[1] = command[1] * 2.0;
  one[2] = command[2] * 0.25;
  for (std::size_t i = 0; i < 3; ++i) {
    one[3 + i] = angular_velocity[i] * 0.25;
    one[6 + i] = projected_gravity[i];
  }
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    one[9 + i] = states[i].position - default_pos[i];
    one[9 + opendoge::kNumJoints + i] = states[i].velocity * 0.05;
    one[9 + opendoge::kNumJoints * 2 + i] = last_action[i];
  }

  std::copy(one.begin(), one.end(), obs.begin());
  std::copy(
    previous.begin(),
    previous.begin() + static_cast<std::ptrdiff_t>(opendoge::kObsDim - opendoge::kOneStepObs),
    obs.begin() + static_cast<std::ptrdiff_t>(opendoge::kOneStepObs));
  return obs;
}

bool safetyFault(
  const std::array<opendoge::MotorState, opendoge::kNumJoints> & states,
  const opendoge::SafetyConfig & safety,
  double now_s,
  std::string & reason)
{
  for (std::size_t i = 0; i < states.size(); ++i) {
    if (!states[i].received) {
      reason = "missing feedback";
      return true;
    }
    if (now_s - states[i].last_feedback_s > safety.state_timeout_s) {
      reason = "feedback timeout";
      return true;
    }
    if (states[i].fault != 0) {
      reason = "motor fault bits";
      return true;
    }
    if (states[i].temperature >= safety.over_temperature_c) {
      reason = "over temperature";
      return true;
    }
  }
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt)) {
    return 1;
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto joints = opendoge::defaultJointMap();
  const auto default_pos = opendoge::defaultJointPosition();
  std::array<opendoge::MotorState, opendoge::kNumJoints> states{};
  std::array<opendoge::MotorCommand, opendoge::kNumJoints> commands{};
  std::array<double, opendoge::kNumJoints> action{};
  std::array<double, opendoge::kNumJoints> last_action{};
  std::array<double, opendoge::kObsDim> obs{};
  std::array<double, opendoge::kNumJoints> target = default_pos;

  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    states[i].position = default_pos[i];
    commands[i] = {default_pos[i], 0.0, 0.0, 0.0, opt.safe_kd};
  }

  auto policy = opendoge::makePolicy(opt.policy_backend);
  if (!policy) {
    std::cerr << "Unknown policy backend: " << opt.policy_backend << "\n";
    return 1;
  }
  std::string error;
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
          can.sendStop(joint, true);
        }
        can.sendMotionMode(joint);
        can.sendEnable(joint);
      }
    }
  }

  std::cout << "OpenDoge deploy running: "
            << (opt.dry_run ? "dry-run" : "real CAN")
            << ", policy=" << opt.policy_backend
            << ", control=" << opt.control_hz << "Hz\n";

  const double start_s = nowSeconds();
  double next_control_s = start_s;
  double next_infer_s = start_s;
  double next_target_s = start_s;
  bool damping = true;
  opendoge::SafetyConfig safety;
  safety.safe_kd = opt.safe_kd;

  while (!g_stop.load()) {
    const double t = nowSeconds();
    if (opt.duration_s > 0.0 && t - start_s >= opt.duration_s) {
      break;
    }

    if (!opt.dry_run) {
      can.drain(states, t);
      if (!can.ok()) {
        damping = true;
        error = can.lastError();
      }
    }

    std::string safety_reason;
    if (!opt.dry_run && safetyFault(states, safety, t, safety_reason)) {
      if (!damping) {
        std::cerr << "Entering damping: " << safety_reason << "\n";
      }
      damping = true;
    }

    if (t >= next_infer_s) {
      const std::array<double, 3> command{0.0, 0.0, 0.0};
      const std::array<double, 3> angular_velocity{0.0, 0.0, 0.0};
      const std::array<double, 3> projected_gravity{0.0, 0.0, -1.0};
      obs = buildObservation(states, default_pos, last_action, command, angular_velocity, projected_gravity, obs);
      if (!policy->infer(obs, action, error)) {
        std::cerr << "Policy infer failed: " << error << "\n";
        damping = true;
      }
      next_infer_s += 1.0 / opt.inference_hz;
    }

    if (t >= next_target_s) {
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        last_action[i] = std::clamp(action[i], -1.0, 1.0);
        target[i] = default_pos[i] + last_action[i] * opt.action_scale;
      }
      next_target_s += 1.0 / opt.target_hz;
    }

    if (t >= next_control_s) {
      for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
        if (damping) {
          commands[i] = {states[i].position, 0.0, 0.0, 0.0, opt.safe_kd};
        } else {
          commands[i] = {target[i], 0.0, 0.0, opt.kp, opt.kd};
        }
        if (!opt.dry_run) {
          can.sendMotion(joints[i], commands[i]);
        }
      }
      next_control_s += 1.0 / opt.control_hz;
    }

    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  if (!opt.dry_run) {
    for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
      opendoge::MotorCommand damp{states[i].position, 0.0, 0.0, 0.0, opt.safe_kd};
      can.sendMotion(joints[i], damp);
    }
    can.close();
  }

  std::cout << "OpenDoge deploy stopped\n";
  return 0;
}
