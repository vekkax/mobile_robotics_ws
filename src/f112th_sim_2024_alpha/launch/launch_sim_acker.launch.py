# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument


from launch_ros.actions import Node

def generate_launch_description():

    

    package_name = 'f112th_sim_2024_alpha'
    rviz_file = 'rviz_model.rviz'


    # Include the robot_state_publisher launch file with sim time enabled
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp_acker.launch.py'
                )]),
                launch_arguments={'use_sim_time': 'true'}.items()  # Passing sim time
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'Alphajor'],
                        output='screen')

    # twist_mux Node
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux', 
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', 'cmd_vel')]
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Rviz Node
    pkg_path = os.path.join(get_package_share_path(package_name))
    default_rviz_config_path = os.path.join(pkg_path + '/description/' + rviz_file)
    rviz_arg = DeclareLaunchArgument(name='rvizmodel', default_value=str(default_rviz_config_path), description='Absolute path to rviz model file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizmodel')],
    )

    # Launch all nodes
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joystick,
        twist_mux_node,
        rviz_arg,
        rviz_node,
    ])
