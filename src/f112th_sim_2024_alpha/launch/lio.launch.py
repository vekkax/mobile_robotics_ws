import os
from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

package_name='f112th_sim_2024_alpha'
robot_localization_file_path = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')


def generate_launch_description():

  # Start robot localization using an Extended Kalman filter
    localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path,  {'use_sim_time': True}]
        )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','launch_sim.launch.py'
        )]))

    Lidar_odom = Node(
        package='lidar_odometry',
        executable='lidar_odometry_node',
        #parameters=[robot_localization_file_path, {'use_sim_time': use_sim_time}]
        #remappings=[('/scan','/Fscan')]
    )


    # Launch them all!
    return LaunchDescription([
        sim,
        Lidar_odom,
        localization,       
    ])