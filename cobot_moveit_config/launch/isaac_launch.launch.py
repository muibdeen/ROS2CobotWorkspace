from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="cobot_moveit_config").to_moveit_configs()
    package_share = get_package_share_directory("cobot_moveit_config")

    use_rviz = LaunchConfiguration("use_rviz")

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "source_list": ["/joint_state_isaac"],
            "rate": 60,
        }],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.controller_connection_timeout": 0.0,
                "publish_planning_scene_hz": 4.0,
                "planning_pipelines": ["ompl", "chomp"],
            },

        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(package_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),  # <-- correct Jazzy syntax
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        joint_state_publisher_node,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
