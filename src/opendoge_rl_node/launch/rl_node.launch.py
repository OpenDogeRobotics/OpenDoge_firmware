from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("policy_path", default_value=""),
            DeclareLaunchArgument("num_dofs", default_value="12"),
            DeclareLaunchArgument("update_rate_hz", default_value="500.0"),
            DeclareLaunchArgument("timeout_state_ms", default_value="100"),
            DeclareLaunchArgument("timeout_imu_ms", default_value="100"),
            DeclareLaunchArgument("safe_kd", default_value="2.0"),
            DeclareLaunchArgument("safe_kp", default_value="0.0"),
            DeclareLaunchArgument("safe_tau", default_value="0.0"),
            DeclareLaunchArgument("command_topic", default_value="/robot_joint_controller/command"),
            DeclareLaunchArgument("state_topic", default_value="/robot_joint_controller/state"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            Node(
                package="opendoge_rl_node",
                executable="rl_node",
                name="opendoge_rl_node",
                output="screen",
                parameters=[
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "num_dofs": LaunchConfiguration("num_dofs"),
                        "update_rate_hz": LaunchConfiguration("update_rate_hz"),
                        "timeout_state_ms": LaunchConfiguration("timeout_state_ms"),
                        "timeout_imu_ms": LaunchConfiguration("timeout_imu_ms"),
                        "safe_kd": LaunchConfiguration("safe_kd"),
                        "safe_kp": LaunchConfiguration("safe_kp"),
                        "safe_tau": LaunchConfiguration("safe_tau"),
                        "command_topic": LaunchConfiguration("command_topic"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                    }
                ],
            ),
        ]
    )

