#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <sched.h>
#include <sys/mman.h>

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
  bool realtime{false};
  int cpu{-1};
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
    << "  --realtime                              try mlockall + SCHED_FIFO\n"
    << "  --cpu N                                 pin process to CPU N\n"
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
      } else if (arg == "--realtime") {
        opt.realtime = true;
      } else if (arg == "--cpu") {
        opt.cpu = std::stoi(need_value(arg));
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

std::string hexValue(std::uint64_t value, int width = 0)
{
  std::ostringstream ss;
  ss << "0x" << std::uppercase << std::hex;
  if (width > 0) {
    ss << std::setw(width) << std::setfill('0');
  }
  ss << value;
  return ss.str();
}

std::string describeBits(std::uint32_t value)
{
  if (value == 0) {
    return "none";
  }
  std::ostringstream ss;
  bool first = true;
  for (int bit = 0; bit < 32; ++bit) {
    if ((value & (1u << bit)) == 0) {
      continue;
    }
    if (!first) {
      ss << ",";
    }
    ss << "bit" << bit;
    first = false;
  }
  return ss.str();
}

void applyRuntimeTuning(const Options & opt)
{
  if (opt.cpu >= 0) {
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(opt.cpu, &mask);
    if (::sched_setaffinity(0, sizeof(mask), &mask) != 0) {
      std::cerr << "Warning: sched_setaffinity failed\n";
    }
  }
  if (opt.realtime) {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      std::cerr << "Warning: mlockall failed\n";
    }
    sched_param param{};
    param.sched_priority = 60;
    if (::sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
      std::cerr << "Warning: sched_setscheduler(SCHED_FIFO) failed\n";
    }
  }
}

/// Compute adaptive gait phase matching UniLab training.
/// cmd_speed = norm([vx, vy, vyaw]); freq ∈ [1.2, 2.5] Hz; phase wraps at 1.0.
/// dt is the time step between phase advances (1.0 / inference_hz).
inline double advancePhase(
  const opendoge::OperatorCommand & command, double phase, double dt)
{
  const double cmd_speed = std::sqrt(
    command.vx * command.vx + command.vy * command.vy + command.yaw_rate * command.yaw_rate);
  const double freq = std::clamp(1.2 + 1.3 * cmd_speed / 0.6, 1.2, 2.5);
  return std::fmod(phase + dt * freq, 1.0);
}

/// Build 49-dim single-frame actor observation matching UniLab Round 26.
/// No privileged information — all 49 dims are deployable on real hardware.
///   gyro(3) + neg_gravity(3) + dof_pos_diff(12) + dof_vel(12)
///   + last_action(12) + commands(3) + feet_phase(4)
std::array<double, opendoge::kObsDim> buildObservation(
  const std::array<opendoge::MotorState, opendoge::kNumJoints> & states,
  const std::array<opendoge::JointCalibration, opendoge::kNumJoints> & calibration,
  const std::array<double, opendoge::kNumJoints> & default_pos,
  const std::array<double, opendoge::kNumJoints> & last_action,
  const opendoge::OperatorCommand & command,
  const opendoge::ImuSample & imu,
  double phase)
{
  std::array<double, opendoge::kObsDim> obs{};
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
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    const auto pos = opendoge::logicalPosition(states[i].position, calibration[i]);
    obs[offset + i] = pos - default_pos[i];
  }
  offset += opendoge::kNumJoints;

  // 4. dof_vel — 12 dims
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    obs[offset + i] = opendoge::logicalVelocity(states[i].velocity, calibration[i]);
  }
  offset += opendoge::kNumJoints;

  // 5. last_action — 12 dims
  for (std::size_t i = 0; i < opendoge::kNumJoints; ++i) {
    obs[offset + i] = last_action[i];
  }
  offset += opendoge::kNumJoints;

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
  // offset += 4;  // final: 49 dims total

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
      reason = joints[i].name + ": status fault bits=" +
        hexValue(states[i].fault, 2) + " [" + describeBits(states[i].fault) + "]";
      return true;
    }
    if (states[i].param_fault != 0) {
      reason = joints[i].name + ": faultSta=" +
        hexValue(states[i].param_fault, 8) + " [" + describeBits(states[i].param_fault) + "]";
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

struct LoopStats
{
  std::uint64_t control_ticks{0};
  std::uint64_t inference_ticks{0};
  std::uint64_t target_ticks{0};
  std::uint64_t missed_control_deadlines{0};
  double max_control_late_s{0.0};

  void resetWindow()
  {
    control_ticks = 0;
    inference_ticks = 0;
    target_ticks = 0;
    missed_control_deadlines = 0;
    max_control_late_s = 0.0;
  }
};

void waitUntilNextDeadline(double next_deadline_s)
{
  const double remaining_s = next_deadline_s - nowSeconds();
  if (remaining_s > 0.0003) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  } else if (remaining_s > 0.00005) {
    std::this_thread::yield();
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
  applyRuntimeTuning(opt);

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
  double next_status_s = start_s + 1.0;
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
  LoopStats loop_stats;
  std::uint64_t last_can_sent = 0;
  std::uint64_t last_can_received = 0;

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
      ++loop_stats.inference_ticks;
      phase = advancePhase(command, phase, 1.0 / config.inference_hz);
      obs = buildObservation(states, config.joints, default_pos, last_action, command, imu, phase);
      if (!policy->infer(obs, action, error)) {
        fault_latched = true;
        fault_reason = "policy infer failed: " + error;
        runtime_state = RuntimeState::DampingFault;
      }
      next_infer_s = t + 1.0 / config.inference_hz;
    }

    if (t >= next_target_s) {
      ++loop_stats.target_ticks;
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
      ++loop_stats.control_ticks;
      const double control_late_s = t - next_control_s;
      loop_stats.max_control_late_s = std::max(loop_stats.max_control_late_s, control_late_s);
      if (control_late_s > 0.0005) {
        ++loop_stats.missed_control_deadlines;
      }
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
      const auto & can_stats = can.stats();
      const auto sent_delta = can_stats.frames_sent - last_can_sent;
      const auto recv_delta = can_stats.frames_received - last_can_received;
      std::cout << "state=" << stateName(runtime_state)
                << " active_cmd=" << (command.active ? 1 : 0)
                << " imu=" << (imu.valid ? 1 : 0)
                << " ctrl_ticks=" << loop_stats.control_ticks
                << " infer_ticks=" << loop_stats.inference_ticks
                << " target_ticks=" << loop_stats.target_ticks
                << " max_late_us=" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6)
                << " missed_ctrl=" << loop_stats.missed_control_deadlines
                << " can_tx=" << sent_delta
                << " can_rx=" << recv_delta
                << " can_err=" << (can_stats.read_errors + can_stats.write_errors);
      if (fault_latched) {
        std::cout << " fault=\"" << fault_reason << "\"";
      }
      std::cout << "\n";
      loop_stats.resetWindow();
      last_can_sent = can_stats.frames_sent;
      last_can_received = can_stats.frames_received;
      next_status_s = t + 1.0;
    }

    const double next_deadline_s = std::min(
      std::min(next_control_s, next_infer_s),
      std::min(std::min(next_target_s, next_fault_poll_s), std::min(next_status_s, next_input_s)));
    waitUntilNextDeadline(next_deadline_s);
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
