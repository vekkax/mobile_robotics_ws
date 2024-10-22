import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Load the URDF and process the robot description
    pkg_path = os.path.join(get_package_share_directory('f112th_sim_2024_alpha'))
    xacro_file = os.path.join(pkg_path, 'description', 'description_ackermann', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Load the controller configuration from YAML
    controller_config = os.path.join(pkg_path, 'config', 'acker_controller.yaml')

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #name='acker_state_publisher',
        output='screen',
        parameters=[params]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
        remappings=[],
    )
    
    # Spawner for bycicle_steering_controller
    robot_bicycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['biclycle_steering_controller', "--param-file", controller_config],
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_bicycle_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'),

        
        control_node,
        robot_state_publisher,
        robot_bicycle_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner

    ])