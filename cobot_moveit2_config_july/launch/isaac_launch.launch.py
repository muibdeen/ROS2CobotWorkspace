from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("cobot_description")
    moveit_config = (
        MoveItConfigsBuilder("cobot_description", package_name="cobot_moveit2_config_july")
        .robot_description(file_path="config/cobot_description.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "stomp"])
        .to_moveit_configs()
    )

    package_share = get_package_share_directory("cobot_moveit2_config_july")
    use_rviz = LaunchConfiguration("use_rviz")

    # Only ONE move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # Fixed: Proper static transform (world → base_link) or remove entirely
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="both",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{
            "source_list": ["/joint_state_isaac"],
            "rate": 10,
        }],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("cobot_moveit2_config_july"), 
        "config", 
        "ros2_controllers.yaml"
    )

    # Fixed: Removed the remapping that causes duplicate description issues
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # Pass description directly
            ros2_controllers_path,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    cobot_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cobot_arm_controller", "-c", "/controller_manager"],
    )

    end_effector_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["probe_controller", "-c", "/controller_manager"],
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
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        rviz_node,
        static_tf,
        robot_state_publisher_node,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_state_publisher_node,
        cobot_arm_controller_spawner,
        end_effector_controller_spawner,
    ])
