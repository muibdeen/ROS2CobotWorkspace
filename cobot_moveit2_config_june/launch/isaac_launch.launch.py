from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("cobot_description", package_name="cobot_moveit2_config_june").to_moveit_configs()
    package_share = get_package_share_directory("cobot_moveit2_config_june")

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
            moveit_config.trajectory_execution,
            {
                "use_sim_time": True,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.controller_connection_timeout": 5.0,
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
    # 1. The Controller Manager Node (Loads the fake hardware)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(package_share, "config", "ros2_controllers.yaml") # Path to your controller config
        ],
        output="screen",
    )

    # 2. Spawner for the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 3. Spawner for the Arm Controller
    cobot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cobot_arm_controller", "--controller-manager", "/controller_manager"],
    )

    # 4. Spawner for the End Effector Controller
    end_effector_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["end_effector_controller", "--controller-manager", "/controller_manager"],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        joint_state_publisher_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        cobot_arm_controller_spawner,
        end_effector_controller_spawner,
        move_group_node,
        rviz_node,
    ])
