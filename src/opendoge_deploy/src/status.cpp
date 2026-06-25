#include "opendoge_deploy/status.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <thread>

namespace opendoge
{

std::string escapeJson(const std::string & input)
{
  std::string out;
  out.reserve(input.size() + 4);
  for (char c : input) {
    switch (c) {
      case '"':  out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out += "\\u00";
          out += "0123456789abcdef"[static_cast<unsigned char>(c) >> 4];
          out += "0123456789abcdef"[static_cast<unsigned char>(c) & 0x0F];
        } else {
          out += c;
        }
    }
  }
  return out;
}

void waitUntilNextDeadline(double next_deadline_s)
{
  namespace chr = std::chrono;
  const double remaining_s = next_deadline_s - chr::duration<double>(
    chr::steady_clock::now().time_since_epoch()).count();
  if (remaining_s > 0.0003) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  } else if (remaining_s > 0.00005) {
    std::this_thread::yield();
  }
}

void emitStatus(
  RuntimeState runtime_state,
  const OperatorCommand & command,
  const std::string & status_file,
  LoopStats & loop_stats,
  std::uint64_t & last_can_sent,
  std::uint64_t & last_can_received,
  const CanStats & can_stats,
  const std::string & fault_reason,
  bool rl_fallback_active,
  const std::array<MotorState, kNumJoints> & states,
  const std::array<JointCalibration, kNumJoints> & calib,
  const std::array<JointMap, kNumJoints> & joints,
  const ImuSample & imu,
  double pc_startup_start_s,
  double pc_startup_ramp_s,
  double t)
{
  const auto sent_delta = can_stats.frames_sent - last_can_sent;
  const auto recv_delta = can_stats.frames_received - last_can_received;
  std::cout << "state=" << stateName(runtime_state)
            << " active_cmd=" << (command.active ? 1 : 0)
            << " pos_ctrl=" << (command.position_control ? 1 : 0)
            << " rl_infer=" << (command.rl_inference ? 1 : 0)
            << " low_gain=" << (command.low_gain_mode ? 1 : 0)
            << " imu=" << (imu.valid ? 1 : 0)
            << " ctrl_ticks=" << loop_stats.control_ticks
            << " infer_ticks=" << loop_stats.inference_ticks
            << " target_ticks=" << loop_stats.target_ticks
            << " max_late_us=" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6)
            << " missed_ctrl=" << loop_stats.missed_control_deadlines
            << " can_tx=" << sent_delta
            << " can_rx=" << recv_delta
            << " can_err=" << (can_stats.read_errors + can_stats.write_errors)
            << " ramp_pct=" << (runtime_state == RuntimeState::EnteringPosition
              ? static_cast<int>(100.0 * (t - pc_startup_start_s) / pc_startup_ramp_s) : 100)
            << " rl_fb=" << (rl_fallback_active ? 1 : 0);
  if (runtime_state == RuntimeState::DampingFault) {
    std::cout << " fault=\"" << fault_reason << "\"";
  }
  std::cout << "\n";

  // Write JSON status snapshot for external consumers (web console, etc.)
  if (!status_file.empty()) {
    std::ofstream sf(status_file + ".tmp");
    if (sf) {
      sf << "{";
      sf << "\"t\":" << t << ",";
      sf << "\"state\":\"" << stateName(runtime_state) << "\",";
      sf << "\"active_cmd\":" << (command.active ? "true" : "false") << ",";
      sf << "\"estop\":" << (command.estop ? "true" : "false") << ",";
      sf << "\"position_control\":" << (command.position_control ? "true" : "false") << ",";
      sf << "\"rl_inference\":" << (command.rl_inference ? "true" : "false") << ",";
      sf << "\"low_gain\":" << (command.low_gain_mode ? "true" : "false") << ",";
      sf << "\"imu_valid\":" << (imu.valid ? "true" : "false") << ",";
      sf << "\"fault_reason\":\""
         << (runtime_state == RuntimeState::DampingFault ? escapeJson(fault_reason) : "")
         << "\",";
      sf << "\"ctrl_ticks\":" << loop_stats.control_ticks << ",";
      sf << "\"infer_ticks\":" << loop_stats.inference_ticks << ",";
      sf << "\"max_late_us\":" << static_cast<int>(loop_stats.max_control_late_s * 1.0e6) << ",";
      sf << "\"missed_ctrl\":" << loop_stats.missed_control_deadlines << ",";
      sf << "\"can_tx\":" << sent_delta << ",";
      sf << "\"can_rx\":" << recv_delta << ",";
      sf << "\"can_err\":" << (can_stats.read_errors + can_stats.write_errors) << ",";
      sf << "\"command\":[" << command.vx << "," << command.vy << "," << command.yaw_rate << "],";
      // Per-joint state
      sf << "\"joints\":[";
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        if (i > 0) sf << ",";
        const double logical_pos = logicalPosition(states[i].position, calib[i]);
        sf << "{\"n\":\"" << joints[i].name << "\","
           << "\"q\":" << logical_pos << ","
           << "\"dq\":" << logicalVelocity(states[i].velocity, calib[i]) << ","
           << "\"tau\":" << states[i].torque << ","
           << "\"temp\":" << states[i].temperature << ","
           << "\"fault\":" << states[i].fault << ","
           << "\"recv\":" << (states[i].received ? "true" : "false") << "}";
      }
      sf << "],";
      // IMU
      sf << "\"imu\":{";
      sf << "\"wx\":" << imu.angular_velocity[0] << ","
         << "\"wy\":" << imu.angular_velocity[1] << ","
         << "\"wz\":" << imu.angular_velocity[2] << ","
         << "\"gx\":" << imu.projected_gravity[0] << ","
         << "\"gy\":" << imu.projected_gravity[1] << ","
         << "\"gz\":" << imu.projected_gravity[2];
      sf << "}";
      sf << "}\n";
      sf.close();
      std::rename((status_file + ".tmp").c_str(), status_file.c_str());
    }
  }

  loop_stats.resetWindow();
  last_can_sent = can_stats.frames_sent;
  last_can_received = can_stats.frames_received;
}

}  // namespace opendoge
