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


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='f112th_sim_2024_alpha' #<--- CHANGE ME
    rviz_file = 'rviz_model.rviz'

    # path to rviz
    pkg_path = os.path.join(get_package_share_path(package_name))
    
    sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_sim.launch.py'
                )])
    )                

    localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','slam_localization_launch.py'
                )])
    ) 

    mapping = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_launch.py'
                )])
    ) 
    nav_planning = Node(package='nav_2024_alpha', 
                    executable='nav_2024_alpha_node',
    )

    nav_control = Node(package='nav_2024_alpha', 
                    executable='nav_control_alpha',
    )

    odom2tf = Node(package='odom_to_tf', 
                    executable='odom_to_tf_node',
    )    

    # Launch them all!
    return LaunchDescription([
        sim,
        #localization,
        mapping
        #nav_planning,
        #nav_control,
        #odom2tf
    ])