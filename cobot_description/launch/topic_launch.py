
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FileContent, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    "we plan to use use_sim_time as a default false so that we dont mess with Nvidia Isaacs clock"
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'true')

    urdf = FileContent(
            PathJoinSubstitution([FindPackageShare('cobot_description'), 'description', 'new_mycobot_pro_320_pi_2022_topic_based.urdf'])
            )

    robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("cobot_controller"),
                "src",
                "cobot_controller_parameters.yaml",
                ]
            )

    spawn_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", 
             "--set-state", "active", "joint_state_broadcaster"]
    )

    spawn_mecanum_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller",
             "--set-state", "active", "cobot_controller"]
    )

    return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value = 'false',
                description='uses the simulation clock if set to true'
                ),
            
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': urdf}],
                arguments=[urdf]),
            
            Node(
                package='rviz2',
                executable='rviz2'
                ),

            Node(
                package='controller_manager',
                executable='ros2_control_node',
                parameters=[robot_controllers],
                output="both"
                ),

            spawn_jsb,
            spawn_mecanum_controller


            ])
