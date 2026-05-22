from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    description_pkg = get_package_share_directory("opendoge_description")
    control_pkg = get_package_share_directory("opendoge_control")
    bringup_pkg = get_package_share_directory("opendoge_bringup")
    rl_pkg = get_package_share_directory("opendoge_rl_node")

    xacro_file = os.path.join(description_pkg, "urdf", "opendoge_apx.urdf.xacro")
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    ros2_control_file = PathJoinSubstitution([control_pkg, "config", "ros2_control.yaml"])
    controllers_file = PathJoinSubstitution([bringup_pkg, "config", "controllers.yaml"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": use_sim_time}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_control_file,
            controllers_file,
            {"robot_description": robot_description_raw, "use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    robot_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_joint_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    rl_node = Node(
        package="opendoge_rl_node",
        executable="rl_node",
        name="opendoge_rl_node",
        output="screen",
        parameters=[os.path.join(rl_pkg, "config", "opendoge_rl.yaml")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock"),
            robot_state_publisher_node,
            controller_manager_node,
            robot_joint_controller_spawner,
            rl_node,
        ]
    )
