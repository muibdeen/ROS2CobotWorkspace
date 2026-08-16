from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_rl = get_package_share_directory("cobot_rl_implement")
    pkg_desc = get_package_share_directory("cobot_description")
    controllers = get_package_share_directory("cobot_rl_implement")

    urdf_file = os.path.join(pkg_desc, "robot.urdf")
    controllers_yaml = os.path.join(controllers, "config", "controllers.yaml")

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    policy_path = os.path.join(pkg_rl, "exported_policy", "actor.pt")
    use_rviz = LaunchConfiguration("use_rviz")

    # 1. Base Infra Nodes
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc}],
    )

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

    # 2. Controller Spawners (Must wait for controller_manager)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
        output="screen"
    )

    # 3. Application Nodes
    rl_policy_node = Node(
        package="cobot_rl_implement",
        executable="cobot_rl",
        name="cobot_policy_node",
        parameters=[{
            "policy_path": policy_path,
            "base_frame": "ground_link",
            "ee_frame": "w_j6",
        }],
        output="screen",
    )

    pointcloud_jogger_node = Node(
        package="cobot_rl_implement",
        executable="pointcloud_jogger",
        name="jogger",
        output="screen",
    )

    json_jogger_node = Node(
        package="cobot_rl_implement",
        executable="json_jogger",
        name="jogger",
        output="screen",
    )

    rviz_config = os.path.join(pkg_rl, "config", "cobot_rl.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    # --- ORDER OF EXECUTION HANDLERS ---

    # Delay the controllers until the main controller_manager_node starts
    delay_controllers_after_cm = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner, position_controller_spawner]
        )
    )

    # Replaces your time.sleep(5) -> Safely delays json_jogger by 5 seconds after controllers are active
    delay_json_jogger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[json_jogger_node]
                )
            ]
        )
    )
    # Replaces your time.sleep(5) -> Safely delays json_jogger by 5 seconds after controllers are active
    delay_pointcloud_jogger = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[pointcloud_jogger_node]
                )
            ]
        )
    )


    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        
        # Start infrastructure immediately
        controller_manager_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rl_policy_node,
        rviz_node,
        
        # Sequenced event handlers
        delay_controllers_after_cm,
        delay_pointcloud_jogger,
    ])
