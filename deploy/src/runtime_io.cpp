#include "opendoge_deploy/runtime_io.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace opendoge
{
namespace
{
std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

bool parseBool(const std::string & value)
{
  const auto lower = [&]() {
    std::string out = value;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return out;
  }();
  return lower == "1" || lower == "true" || lower == "yes" || lower == "on" || lower == "active";
}

std::unordered_map<std::string, std::string> readKeyValueFile(const std::string & path, std::string & error)
{
  std::unordered_map<std::string, std::string> values;
  std::ifstream file(path);
  if (!file) {
    error = "cannot open " + path;
    return values;
  }

  std::string line;
  int line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.resize(comment);
    }
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto equal = line.find('=');
    if (equal == std::string::npos) {
      error = path + ":" + std::to_string(line_no) + ": expected key=value";
      values.clear();
      return values;
    }
    values[trim(line.substr(0, equal))] = trim(line.substr(equal + 1));
  }
  return values;
}

double getDouble(
  const std::unordered_map<std::string, std::string> & values, const std::string & key, double fallback)
{
  const auto it = values.find(key);
  if (it == values.end() || it->second.empty()) {
    return fallback;
  }
  return std::stod(it->second);
}

bool getBool(
  const std::unordered_map<std::string, std::string> & values, const std::string & key, bool fallback)
{
  const auto it = values.find(key);
  if (it == values.end() || it->second.empty()) {
    return fallback;
  }
  return parseBool(it->second);
}

std::vector<double> parseNumbers(const std::string & text)
{
  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::stringstream ss(normalized);
  std::vector<double> out;
  double value = 0.0;
  while (ss >> value) {
    out.push_back(value);
  }
  return out;
}
}  // namespace

bool loadDeployConfig(
  const std::string & path, const std::array<JointMap, kNumJoints> & joints,
  DeployConfig & config, std::string & error)
{
  error.clear();
  config.joints = defaultJointCalibration();
  if (path.empty()) {
    return true;
  }

  auto values = readKeyValueFile(path, error);
  if (!error.empty()) {
    return false;
  }

  try {
    config.inference_hz = getDouble(values, "inference_hz", config.inference_hz);
    config.target_hz = getDouble(values, "target_hz", config.target_hz);
    config.control_hz = getDouble(values, "control_hz", config.control_hz);
    config.kp = getDouble(values, "kp", config.kp);
    config.kd = getDouble(values, "kd", config.kd);
    config.safe_kd = getDouble(values, "safe_kd", config.safe_kd);
    config.action_scale = getDouble(values, "action_scale", config.action_scale);
    config.state_timeout_s = getDouble(values, "state_timeout_s", config.state_timeout_s);
    config.over_temperature_c = getDouble(values, "over_temperature_c", config.over_temperature_c);
    config.fault_poll_hz = getDouble(values, "fault_poll_hz", config.fault_poll_hz);
    config.pc_startup_ramp_s = getDouble(values, "pc_startup_ramp_s", config.pc_startup_ramp_s);
    config.pc_startup_max_deviation = getDouble(values, "pc_startup_max_deviation", config.pc_startup_max_deviation);
    config.torque_threshold = getDouble(values, "torque_threshold", config.torque_threshold);
    config.torque_timeout_s = getDouble(values, "torque_timeout_s", config.torque_timeout_s);
    config.tracking_error_threshold = getDouble(values, "tracking_error_threshold", config.tracking_error_threshold);
    config.tracking_error_timeout_s = getDouble(values, "tracking_error_timeout_s", config.tracking_error_timeout_s);
    config.command_timeout_s = getDouble(values, "command_timeout_s", config.command_timeout_s);
    config.fall_gravity_z_threshold = getDouble(values, "fall_gravity_z_threshold", config.fall_gravity_z_threshold);
    config.fall_timeout_s = getDouble(values, "fall_timeout_s", config.fall_timeout_s);
    config.command_smoothing_alpha = getDouble(values, "command_smoothing_alpha", config.command_smoothing_alpha);
    config.feedback_wait_timeout_s = getDouble(values, "feedback_wait_timeout_s", config.feedback_wait_timeout_s);
    config.temp_warn_c = getDouble(values, "temp_warn_c", config.temp_warn_c);
    {
      const auto it = values.find("imu_debounce_count");
      if (it != values.end() && !it->second.empty()) {
        config.imu_debounce_count = std::stoi(it->second);
      }
    }

    for (std::size_t i = 0; i < joints.size(); ++i) {
      const auto prefix = "joint." + joints[i].name + ".";
      auto & joint = config.joints[i];
      joint.direction = getDouble(values, prefix + "direction", joint.direction);
      joint.offset = getDouble(values, prefix + "offset", joint.offset);
      joint.lower = getDouble(values, prefix + "lower", joint.lower);
      joint.upper = getDouble(values, prefix + "upper", joint.upper);
      joint.max_position_step = getDouble(values, prefix + "max_position_step", joint.max_position_step);
      joint.max_velocity = getDouble(values, prefix + "max_velocity", joint.max_velocity);
      joint.max_torque = getDouble(values, prefix + "max_torque", joint.max_torque);
      joint.max_kp = getDouble(values, prefix + "max_kp", joint.max_kp);
      joint.max_kd = getDouble(values, prefix + "max_kd", joint.max_kd);
      joint.direction = joint.direction < 0.0 ? -1.0 : 1.0;
      if (joint.lower > joint.upper) {
        std::swap(joint.lower, joint.upper);
      }
    }
  } catch (const std::exception & exc) {
    error = "bad deploy config " + path + ": " + exc.what();
    return false;
  }
  return true;
}

bool readCommandFile(const std::string & path, OperatorCommand & command, std::string & error)
{
  error.clear();
  if (path.empty()) {
    return true;
  }

  auto values = readKeyValueFile(path, error);
  if (!error.empty()) {
    std::ifstream file(path);
    if (!file) {
      return false;
    }
    std::string line;
    std::getline(file, line);
    const auto nums = parseNumbers(line);
    if (nums.size() < 3) {
      error = "command file must contain vx vy yaw_rate [active] [estop]";
      return false;
    }
    command.vx = nums[0];
    command.vy = nums[1];
    command.yaw_rate = nums[2];
    if (nums.size() > 3) {
      command.active = nums[3] != 0.0;
    }
    if (nums.size() > 4) {
      command.estop = nums[4] != 0.0;
    }
    if (nums.size() > 5) {
      command.position_control = nums[5] != 0.0;
    }
    if (nums.size() > 6) {
      command.rl_inference = nums[6] != 0.0;
    }
    if (nums.size() > 7) {
      command.clear_fault = nums[7] != 0.0;
    }
    if (nums.size() > 8) {
      command.low_gain_mode = nums[8] != 0.0;
    }
    error.clear();
    return true;
  }

  try {
    command.vx = getDouble(values, "vx", command.vx);
    command.vy = getDouble(values, "vy", command.vy);
    command.yaw_rate = getDouble(values, "yaw_rate", command.yaw_rate);
    command.active = getBool(values, "active", command.active);
    command.estop = getBool(values, "estop", command.estop);
    command.position_control = getBool(values, "position_control", command.position_control);
    command.rl_inference = getBool(values, "rl_inference", command.rl_inference);
    command.clear_fault = getBool(values, "clear_fault", command.clear_fault);
    command.low_gain_mode = getBool(values, "low_gain_mode", command.low_gain_mode);
  } catch (const std::exception & exc) {
    error = "bad command file " + path + ": " + exc.what();
    return false;
  }
  return true;
}

bool readImuFile(const std::string & path, ImuSample & imu, double now_s, std::string & error)
{
  error.clear();
  if (path.empty()) {
    return true;
  }

  auto values = readKeyValueFile(path, error);
  if (!error.empty()) {
    std::ifstream file(path);
    if (!file) {
      return false;
    }
    std::string line;
    std::getline(file, line);
    const auto nums = parseNumbers(line);
    if (nums.size() < 6) {
      error = "imu file must contain wx wy wz gx gy gz";
      return false;
    }
    imu.angular_velocity = {nums[0], nums[1], nums[2]};
    imu.projected_gravity = {nums[3], nums[4], nums[5]};
    imu.valid = true;
    imu.last_update_s = now_s;
    error.clear();
    return true;
  }

  try {
    imu.angular_velocity = {
      getDouble(values, "wx", imu.angular_velocity[0]),
      getDouble(values, "wy", imu.angular_velocity[1]),
      getDouble(values, "wz", imu.angular_velocity[2])};
    imu.projected_gravity = {
      getDouble(values, "gx", imu.projected_gravity[0]),
      getDouble(values, "gy", imu.projected_gravity[1]),
      getDouble(values, "gz", imu.projected_gravity[2])};
    imu.valid = true;
    imu.last_update_s = now_s;
  } catch (const std::exception & exc) {
    error = "bad imu file " + path + ": " + exc.what();
    return false;
  }
  return true;
}

}  // namespace opendoge
