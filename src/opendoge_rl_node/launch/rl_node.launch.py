from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("opendoge_rl_node"), "config", "opendoge_rl.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("policy_path", default_value=""),
            DeclareLaunchArgument("policy_backend", default_value="none"),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_state"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("joy_topic", default_value="/joy"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("joint_target_topic", default_value="/joint_target"),
            DeclareLaunchArgument("publish_rate_hz", default_value="200.0"),
            DeclareLaunchArgument("inference_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("control_rate_hz", default_value="1000.0"),
            DeclareLaunchArgument("timeout_state_ms", default_value="100"),
            DeclareLaunchArgument("timeout_imu_ms", default_value="100"),
            Node(
                package="opendoge_rl_node",
                executable="rl_node",
                name="opendoge_rl_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "policy_path": LaunchConfiguration("policy_path"),
                        "policy_backend": LaunchConfiguration("policy_backend"),
                        "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "joy_topic": LaunchConfiguration("joy_topic"),
                        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                        "joint_target_topic": LaunchConfiguration("joint_target_topic"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "inference_rate_hz": LaunchConfiguration("inference_rate_hz"),
                        "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                        "timeout_state_ms": LaunchConfiguration("timeout_state_ms"),
                        "timeout_imu_ms": LaunchConfiguration("timeout_imu_ms"),
                    }
                ],
            ),
        ]
    )
