#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/motor_command.hpp"
#include "robot_msgs/msg/motor_state.hpp"

using namespace std::chrono_literals;

class RlNode : public rclcpp::Node
{
public:
  RlNode() : Node("opendoge_rl_node")
  {
    // Parameters
    num_dofs_ = this->declare_parameter<int>("num_dofs", 12);
    update_rate_hz_ = this->declare_parameter<double>("update_rate_hz", 500.0);
    timeout_state_ms_ = this->declare_parameter<int>("timeout_state_ms", 100);
    timeout_imu_ms_ = this->declare_parameter<int>("timeout_imu_ms", 100);
    safe_kd_ = this->declare_parameter<double>("safe_kd", 2.0);
    safe_kp_ = this->declare_parameter<double>("safe_kp", 0.0);
    safe_tau_ = this->declare_parameter<double>("safe_tau", 0.0);
    policy_path_ = this->declare_parameter<std::string>("policy_path", "");
    command_topic_ = this->declare_parameter<std::string>("command_topic", "/robot_joint_controller/command");
    state_topic_ = this->declare_parameter<std::string>("state_topic", "/robot_joint_controller/state");
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");

    RCLCPP_INFO(this->get_logger(), "opendoge_rl_node starting. policy_path=%s", policy_path_.c_str());

    state_sub_ = this->create_subscription<robot_msgs::msg::RobotState>(
      state_topic_, rclcpp::QoS(10),
      [this](const robot_msgs::msg::RobotState::SharedPtr msg) {
        last_state_ = msg;
        last_state_stamp_ = now();
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::QoS(50),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = msg;
        last_imu_stamp_ = now();
      });

    cmd_pub_ = this->create_publisher<robot_msgs::msg::RobotCommand>(command_topic_, rclcpp::QoS(10));

    auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RlNode::controlLoop, this));
  }

private:
  void controlLoop()
  {
    const auto now_time = now();
    const bool state_ok = last_state_ && (now_time - last_state_stamp_) < rclcpp::Duration::from_seconds(timeout_state_ms_ / 1000.0);
    const bool imu_ok = last_imu_ && (now_time - last_imu_stamp_) < rclcpp::Duration::from_seconds(timeout_imu_ms_ / 1000.0);

    if (!state_ok || !imu_ok) {
      publishSafeCommand();
      if (!state_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "RobotState timeout");
      }
      if (!imu_ok) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IMU timeout");
      }
      return;
    }

    // TODO: Replace with real policy inference using policy_path_
    publishHoldPositionCommand(*last_state_);
  }

  void publishSafeCommand()
  {
    robot_msgs::msg::RobotCommand cmd;
    cmd.motor_command.resize(num_dofs_);
    for (int i = 0; i < num_dofs_; ++i) {
      cmd.motor_command[i].q = 0.0;
      cmd.motor_command[i].dq = 0.0;
      cmd.motor_command[i].tau = safe_tau_;
      cmd.motor_command[i].kp = safe_kp_;
      cmd.motor_command[i].kd = safe_kd_;
    }
    cmd_pub_->publish(cmd);
  }

  void publishHoldPositionCommand(const robot_msgs::msg::RobotState & state)
  {
    robot_msgs::msg::RobotCommand cmd;
    const auto & motors = state.motor_state;
    if (static_cast<int>(motors.size()) < num_dofs_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Motor state size (%zu) < num_dofs (%d)", motors.size(), num_dofs_);
      cmd.motor_command.resize(num_dofs_);
      for (int i = 0; i < num_dofs_; ++i) {
        cmd.motor_command[i].q = 0.0;
        cmd.motor_command[i].dq = 0.0;
        cmd.motor_command[i].tau = 0.0;
        cmd.motor_command[i].kp = safe_kp_;
        cmd.motor_command[i].kd = safe_kd_;
      }
      cmd_pub_->publish(cmd);
      return;
    }

    cmd.motor_command.resize(num_dofs_);
    for (int i = 0; i < num_dofs_; ++i) {
      cmd.motor_command[i].q = motors[i].q;        // hold current position
      cmd.motor_command[i].dq = 0.0;
      cmd.motor_command[i].tau = 0.0;
      cmd.motor_command[i].kp = safe_kp_;
      cmd.motor_command[i].kd = safe_kd_;
    }

    cmd_pub_->publish(cmd);
  }

  // Members
  int num_dofs_;
  double update_rate_hz_;
  int timeout_state_ms_;
  int timeout_imu_ms_;
  double safe_kd_;
  double safe_kp_;
  double safe_tau_;
  std::string policy_path_;
  std::string command_topic_;
  std::string state_topic_;
  std::string imu_topic_;

  rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  robot_msgs::msg::RobotState::SharedPtr last_state_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RlNode>());
  rclcpp::shutdown();
  return 0;
}

