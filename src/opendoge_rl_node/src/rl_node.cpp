#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace
{
double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

std::vector<double> sizedVector(const std::vector<double> & input, std::size_t size, double fallback)
{
  std::vector<double> output = input;
  output.resize(size, fallback);
  return output;
}

std::vector<int64_t> identityMap(std::size_t size)
{
  std::vector<int64_t> map(size);
  std::iota(map.begin(), map.end(), 0);
  return map;
}

std::vector<double> parseLinearPolicyLine(const std::string & line)
{
  std::vector<double> values;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}
}  // namespace

class RlNode : public rclcpp::Node
{
public:
  RlNode()
  : Node("opendoge_rl_node")
  {
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    loadParameters();
    loadPolicy();
    resetState();
    createRosInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "OpenDoge RL node ready: inference %.1f Hz, joint_target %.1f Hz, joints=%zu, backend=%s",
      inference_rate_hz_, publish_rate_hz_, joint_names_.size(), policy_backend_.c_str());
  }

private:
  enum class Mode
  {
    Passive,
    Standby,
    Running,
    Fault
  };

  void loadParameters()
  {
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_state");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    joint_target_topic_ = declare_parameter<std::string>("joint_target_topic", "/joint_target");
    observation_topic_ = declare_parameter<std::string>("observation_topic", "/rl_observation");
    action_topic_ = declare_parameter<std::string>("action_topic", "/rl_action");

    policy_backend_ = declare_parameter<std::string>("policy_backend", "none");
    policy_path_ = declare_parameter<std::string>("policy_path", "");
    transport_mode_ = declare_parameter<std::string>("transport_mode", "ros_topic");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 200.0);
    inference_rate_hz_ = declare_parameter<double>("inference_rate_hz", 50.0);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 1000.0);
    timeout_state_ms_ = declare_parameter<int>("timeout_state_ms", 100);
    timeout_imu_ms_ = declare_parameter<int>("timeout_imu_ms", 100);
    num_single_obs_ = declare_parameter<int>("num_single_obs", 45);
    frame_stack_ = declare_parameter<int>("frame_stack", 6);
    clip_obs_ = declare_parameter<double>("clip_obs", 100.0);
    command_scale_x_ = declare_parameter<double>("command_scale_x", 2.0);
    command_scale_y_ = declare_parameter<double>("command_scale_y", 2.0);
    command_scale_yaw_ = declare_parameter<double>("command_scale_yaw", 0.25);
    obs_scale_ang_vel_ = declare_parameter<double>("obs_scale_ang_vel", 0.25);
    obs_scale_dof_pos_ = declare_parameter<double>("obs_scale_dof_pos", 1.0);
    obs_scale_dof_vel_ = declare_parameter<double>("obs_scale_dof_vel", 0.05);
    action_scale_ = declare_parameter<double>("action_scale", 0.30);
    standby_action_scale_ = declare_parameter<double>("standby_action_scale", 0.05);
    joy_deadzone_ = declare_parameter<double>("joy_deadzone", 0.08);
    max_cmd_x_ = declare_parameter<double>("max_cmd_x", 0.45);
    max_cmd_y_ = declare_parameter<double>("max_cmd_y", 0.20);
    max_cmd_yaw_ = declare_parameter<double>("max_cmd_yaw", 1.20);
    kp_ = declare_parameter<double>("kp", 12.0);
    kd_ = declare_parameter<double>("kd", 0.5);
    safe_kp_ = declare_parameter<double>("safe_kp", 0.0);
    safe_kd_ = declare_parameter<double>("safe_kd", 2.0);

    axis_x_ = declare_parameter<int>("joy_axis_x", 1);
    axis_y_ = declare_parameter<int>("joy_axis_y", 0);
    axis_yaw_ = declare_parameter<int>("joy_axis_yaw", 3);
    btn_deadman_ = declare_parameter<int>("joy_button_deadman", 4);
    btn_standby_ = declare_parameter<int>("joy_button_standby", 7);
    btn_running_ = declare_parameter<int>("joy_button_running", 5);
    btn_passive_ = declare_parameter<int>("joy_button_passive", 6);
    btn_estop_ = declare_parameter<int>("joy_button_estop", 1);

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"});

    const auto dofs = joint_names_.size();
    default_pos_ = sizedVector(declare_parameter<std::vector<double>>("default_pos", std::vector<double>{}), dofs, 0.0);
    lower_limits_ = sizedVector(declare_parameter<std::vector<double>>("lower_limits", std::vector<double>{}), dofs, -2.5);
    upper_limits_ = sizedVector(declare_parameter<std::vector<double>>("upper_limits", std::vector<double>{}), dofs, 2.5);
    action_lower_ = sizedVector(declare_parameter<std::vector<double>>("action_lower", std::vector<double>{}), dofs, -1.0);
    action_upper_ = sizedVector(declare_parameter<std::vector<double>>("action_upper", std::vector<double>{}), dofs, 1.0);
    motor_direction_ = declare_parameter<std::vector<int64_t>>("motor_direction", std::vector<int64_t>{});
    output_map_ = declare_parameter<std::vector<int64_t>>("output_map", std::vector<int64_t>{});
    if (motor_direction_.size() != dofs) {
      motor_direction_.assign(dofs, 1);
    }
    if (output_map_.size() != dofs) {
      output_map_ = identityMap(dofs);
    }

    if (publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 200.0;
    }
    if (inference_rate_hz_ <= 0.0) {
      inference_rate_hz_ = 50.0;
    }
    publish_stride_ = std::max(1, static_cast<int>(std::lround(publish_rate_hz_ / inference_rate_hz_)));
    publish_rate_hz_ = inference_rate_hz_ * publish_stride_;

    if (transport_mode_ != "ros_topic") {
      RCLCPP_WARN(
        get_logger(),
        "transport_mode=%s requested; this package currently publishes ROS2 joint_target and leaves LCM/DDS adapter integration to the motor bridge.",
        transport_mode_.c_str());
    }
  }

  void resetState()
  {
    const auto dofs = joint_names_.size();
    joint_pos_.assign(dofs, 0.0);
    joint_vel_.assign(dofs, 0.0);
    target_pos_ = default_pos_;
    action_.assign(dofs, 0.0);
    observation_.assign(static_cast<std::size_t>(num_single_obs_), 0.0);
    stacked_observation_.assign(static_cast<std::size_t>(num_single_obs_ * frame_stack_), 0.0);
    obs_history_.assign(static_cast<std::size_t>(frame_stack_), observation_);
  }

  void createRosInterfaces()
  {
    rclcpp::SubscriptionOptions sensor_options;
    sensor_options.callback_group = sensor_group_;
    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_group_;

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RlNode::jointStateCallback, this, std::placeholders::_1), sensor_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RlNode::imuCallback, this, std::placeholders::_1), sensor_options);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, rclcpp::QoS(20),
      std::bind(&RlNode::joyCallback, this, std::placeholders::_1), command_options);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(20),
      std::bind(&RlNode::cmdVelCallback, this, std::placeholders::_1), command_options);

    target_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_target_topic_, rclcpp::QoS(20));
    obs_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(observation_topic_, rclcpp::QoS(10));
    action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(action_topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RlNode::publishLoop, this), control_group_);
  }

  void loadPolicy()
  {
    if (policy_backend_ == "none") {
      RCLCPP_WARN(get_logger(), "policy_backend=none: target will hold default_pos until RKNN/ONNX backend is added.");
      return;
    }

    if (policy_backend_ == "linear_csv") {
      loadLinearCsvPolicy();
      return;
    }

    if (policy_backend_ == "rknn") {
      RCLCPP_WARN(
        get_logger(),
        "policy_backend=rknn selected, but RKNN runtime is not linked in this build. Convert this node backend after installing Rockchip rknnrt on RK3588.");
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown policy_backend=%s; using default_pos output.", policy_backend_.c_str());
  }

  void loadLinearCsvPolicy()
  {
    if (policy_path_.empty()) {
      RCLCPP_WARN(get_logger(), "linear_csv policy requested without policy_path.");
      return;
    }
    std::ifstream file(policy_path_);
    if (!file) {
      RCLCPP_WARN(get_logger(), "Cannot open policy_path=%s.", policy_path_.c_str());
      return;
    }

    linear_weights_.clear();
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      linear_weights_.push_back(parseLinearPolicyLine(line));
    }

    const auto expected_cols = static_cast<std::size_t>(num_single_obs_ * frame_stack_ + 1);
    if (linear_weights_.size() != joint_names_.size()) {
      RCLCPP_WARN(get_logger(), "linear_csv rows must equal joint count; got %zu.", linear_weights_.size());
      linear_weights_.clear();
      return;
    }
    for (const auto & row : linear_weights_) {
      if (row.size() != expected_cols) {
        RCLCPP_WARN(get_logger(), "linear_csv columns must be obs_dim+1=%zu.", expected_cols);
        linear_weights_.clear();
        return;
      }
    }

    RCLCPP_INFO(get_logger(), "Loaded linear_csv policy: %zu outputs, %zu inputs.", linear_weights_.size(), expected_cols - 1);
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (msg->position.size() < joint_names_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_state position length is too small.");
      return;
    }

    if (msg->name.size() >= joint_names_.size()) {
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it == msg->name.end()) {
          continue;
        }
        const auto src = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
        joint_pos_[i] = msg->position[src] * static_cast<double>(motor_direction_[i]);
        if (msg->velocity.size() > src) {
          joint_vel_[i] = msg->velocity[src] * static_cast<double>(motor_direction_[i]);
        }
      }
    } else {
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        joint_pos_[i] = msg->position[i] * static_cast<double>(motor_direction_[i]);
        if (msg->velocity.size() > i) {
          joint_vel_[i] = msg->velocity[i] * static_cast<double>(motor_direction_[i]);
        }
      }
    }

    state_received_ = true;
    last_state_stamp_ = now();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    quat_x_ = msg->orientation.x;
    quat_y_ = msg->orientation.y;
    quat_z_ = msg->orientation.z;
    quat_w_ = msg->orientation.w;
    base_ang_vel_[0] = msg->angular_velocity.x;
    base_ang_vel_[1] = msg->angular_velocity.y;
    base_ang_vel_[2] = msg->angular_velocity.z;
    imu_received_ = true;
    last_imu_stamp_ = now();
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_[0] = clamp(msg->linear.x, -max_cmd_x_, max_cmd_x_);
    command_[1] = clamp(msg->linear.y, -max_cmd_y_, max_cmd_y_);
    command_[2] = clamp(msg->angular.z, -max_cmd_yaw_, max_cmd_yaw_);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto axis = [&](int index) {
      if (index < 0 || index >= static_cast<int>(msg->axes.size())) {
        return 0.0;
      }
      const auto value = static_cast<double>(msg->axes[index]);
      return std::abs(value) < joy_deadzone_ ? 0.0 : value;
    };
    const auto button = [&](int index) {
      return index >= 0 && index < static_cast<int>(msg->buttons.size()) && msg->buttons[index] != 0;
    };

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      const bool deadman = btn_deadman_ < 0 || button(btn_deadman_);
      command_[0] = deadman ? clamp(axis(axis_x_) * max_cmd_x_, -max_cmd_x_, max_cmd_x_) : 0.0;
      command_[1] = deadman ? clamp(axis(axis_y_) * max_cmd_y_, -max_cmd_y_, max_cmd_y_) : 0.0;
      command_[2] = deadman ? clamp(axis(axis_yaw_) * max_cmd_yaw_, -max_cmd_yaw_, max_cmd_yaw_) : 0.0;
    }

    if (button(btn_estop_)) {
      mode_ = Mode::Fault;
    } else if (button(btn_passive_)) {
      mode_ = Mode::Passive;
    } else if (button(btn_standby_) && state_received_ && imu_received_) {
      mode_ = Mode::Standby;
    } else if (button(btn_running_) && state_received_ && imu_received_) {
      mode_ = Mode::Running;
    }
  }

  void publishLoop()
  {
    const bool should_infer = publish_tick_++ % publish_stride_ == 0;
    if (should_infer) {
      inferenceLoop();
    }
    publishTarget();
  }

  void inferenceLoop()
  {
    if (!inputsHealthy()) {
      mode_ = mode_ == Mode::Fault ? Mode::Fault : Mode::Passive;
      action_.assign(action_.size(), 0.0);
      target_pos_ = default_pos_;
      return;
    }

    updateObservation();
    updateAction();
    updateTargetFromAction();
    publishDebugVectors();
  }

  bool inputsHealthy()
  {
    const auto current = now();
    const bool state_ok = state_received_ &&
      (current - last_state_stamp_) < rclcpp::Duration::from_seconds(timeout_state_ms_ / 1000.0);
    const bool imu_ok = imu_received_ &&
      (current - last_imu_stamp_) < rclcpp::Duration::from_seconds(timeout_imu_ms_ / 1000.0);

    if (!state_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_state timeout.");
    }
    if (!imu_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU timeout.");
    }
    return state_ok && imu_ok;
  }

  void updateObservation()
  {
    std::fill(observation_.begin(), observation_.end(), 0.0);
    std::vector<double> pos;
    std::vector<double> vel;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pos = joint_pos_;
      vel = joint_vel_;
    }

    std::array<double, 3> cmd{};
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      cmd = command_;
    }

    std::array<double, 3> ang_vel{};
    std::array<double, 3> projected_gravity{};
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      ang_vel = base_ang_vel_;
      projected_gravity = quatToProjectedGravity();
    }

    auto write = [&](std::size_t index, double value) {
      if (index < observation_.size()) {
        observation_[index] = clamp(value, -clip_obs_, clip_obs_);
      }
    };

    write(0, mode_ == Mode::Running ? cmd[0] * command_scale_x_ : 0.0);
    write(1, mode_ == Mode::Running ? cmd[1] * command_scale_y_ : 0.0);
    write(2, mode_ == Mode::Running ? cmd[2] * command_scale_yaw_ : 0.0);

    for (std::size_t i = 0; i < 3; ++i) {
      write(3 + i, ang_vel[i] * obs_scale_ang_vel_);
      write(6 + i, projected_gravity[i]);
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      write(9 + i, (pos[i] - default_pos_[i]) * obs_scale_dof_pos_);
      write(9 + joint_names_.size() + i, vel[i] * obs_scale_dof_vel_);
      write(9 + joint_names_.size() * 2 + i, action_[i]);
    }

    obs_history_.erase(obs_history_.begin());
    obs_history_.push_back(observation_);

    for (std::size_t frame = 0; frame < obs_history_.size(); ++frame) {
      std::copy(
        obs_history_[frame].begin(), obs_history_[frame].end(),
        stacked_observation_.begin() + static_cast<std::ptrdiff_t>(frame * observation_.size()));
    }
  }

  std::array<double, 3> quatToProjectedGravity() const
  {
    const double gx = 0.0;
    const double gy = 0.0;
    const double gz = -1.0;

    const double x = quat_x_;
    const double y = quat_y_;
    const double z = quat_z_;
    const double w = quat_w_;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);
    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    return {
      r00 * gx + r10 * gy + r20 * gz,
      r01 * gx + r11 * gy + r21 * gz,
      r02 * gx + r12 * gy + r22 * gz};
  }

  void updateAction()
  {
    if (mode_ == Mode::Passive || mode_ == Mode::Fault) {
      action_.assign(action_.size(), 0.0);
      return;
    }

    if (policy_backend_ == "linear_csv" && !linear_weights_.empty()) {
      for (std::size_t out = 0; out < action_.size(); ++out) {
        double value = linear_weights_[out].back();
        for (std::size_t i = 0; i < stacked_observation_.size(); ++i) {
          value += linear_weights_[out][i] * stacked_observation_[i];
        }
        action_[out] = clamp(value, action_lower_[out], action_upper_[out]);
      }
      return;
    }

    action_.assign(action_.size(), 0.0);
  }

  void updateTargetFromAction()
  {
    const double scale = mode_ == Mode::Running ? action_scale_ : standby_action_scale_;
    for (std::size_t actual = 0; actual < joint_names_.size(); ++actual) {
      const auto policy_index = output_map_[actual];
      const double raw_action =
        policy_index >= 0 && policy_index < static_cast<int64_t>(action_.size()) ?
        action_[static_cast<std::size_t>(policy_index)] : 0.0;
      const double directed = raw_action * static_cast<double>(motor_direction_[actual]);
      target_pos_[actual] = clamp(default_pos_[actual] + directed * scale, lower_limits_[actual], upper_limits_[actual]);
    }
  }

  void publishTarget()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position = target_pos_;
    msg.velocity.assign(joint_names_.size(), 0.0);
    msg.effort.assign(joint_names_.size(), mode_ == Mode::Passive || mode_ == Mode::Fault ? safe_kp_ : kp_);
    target_pub_->publish(msg);

    (void)control_rate_hz_;
    (void)kd_;
    (void)safe_kd_;
  }

  void publishDebugVectors()
  {
    std_msgs::msg::Float64MultiArray obs_msg;
    obs_msg.data = stacked_observation_;
    obs_pub_->publish(obs_msg);

    std_msgs::msg::Float64MultiArray action_msg;
    action_msg.data = action_;
    action_pub_->publish(action_msg);
  }

  std::string joint_state_topic_;
  std::string imu_topic_;
  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string joint_target_topic_;
  std::string observation_topic_;
  std::string action_topic_;
  std::string policy_backend_;
  std::string policy_path_;
  std::string transport_mode_;

  double publish_rate_hz_{200.0};
  double inference_rate_hz_{50.0};
  double control_rate_hz_{1000.0};
  int timeout_state_ms_{100};
  int timeout_imu_ms_{100};
  int num_single_obs_{45};
  int frame_stack_{6};
  int publish_stride_{4};
  int publish_tick_{0};
  double clip_obs_{100.0};
  double command_scale_x_{2.0};
  double command_scale_y_{2.0};
  double command_scale_yaw_{0.25};
  double obs_scale_ang_vel_{0.25};
  double obs_scale_dof_pos_{1.0};
  double obs_scale_dof_vel_{0.05};
  double action_scale_{0.30};
  double standby_action_scale_{0.05};
  double joy_deadzone_{0.08};
  double max_cmd_x_{0.45};
  double max_cmd_y_{0.20};
  double max_cmd_yaw_{1.20};
  double kp_{12.0};
  double kd_{0.5};
  double safe_kp_{0.0};
  double safe_kd_{2.0};

  int axis_x_{1};
  int axis_y_{0};
  int axis_yaw_{3};
  int btn_deadman_{4};
  int btn_standby_{7};
  int btn_running_{5};
  int btn_passive_{6};
  int btn_estop_{1};

  std::vector<std::string> joint_names_;
  std::vector<double> default_pos_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> action_lower_;
  std::vector<double> action_upper_;
  std::vector<int64_t> motor_direction_;
  std::vector<int64_t> output_map_;
  std::vector<std::vector<double>> linear_weights_;

  std::vector<double> joint_pos_;
  std::vector<double> joint_vel_;
  std::vector<double> target_pos_;
  std::vector<double> action_;
  std::vector<double> observation_;
  std::vector<double> stacked_observation_;
  std::vector<std::vector<double>> obs_history_;
  std::array<double, 3> command_{0.0, 0.0, 0.0};
  std::array<double, 3> base_ang_vel_{0.0, 0.0, 0.0};
  double quat_x_{0.0};
  double quat_y_{0.0};
  double quat_z_{0.0};
  double quat_w_{1.0};

  std::mutex state_mutex_;
  std::mutex imu_mutex_;
  std::mutex command_mutex_;
  bool state_received_{false};
  bool imu_received_{false};
  Mode mode_{Mode::Passive};
  rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RlNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
