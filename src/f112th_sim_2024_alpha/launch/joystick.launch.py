import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    package_name='f112th_sim_2024_alpha' #<--- CHANGE ME

    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    teleop_node = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': False}],
                    remappings=[('/cmd_vel_out','cmd_vel')]
    )

    TTC_node = Node(package='time_to_collision',
                    executable='time_to_collision_node'
    )

    Wall_following = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('wall_following_alpha'),'launch','wall_following.launch.py'
                )])
    )

    scan_node = Node(package='scan_pkg', 
                    executable='scan_node',
    )    

    FTG_node = Node(package='follow_the_gap', 
                    executable='follow_the_gap_node',
    ) 
    # Launch them all!
    return LaunchDescription([
        joy_node,
        teleop_node,
        twist_mux_node,
        #Wall_following,
        FTG_node,
        TTC_node,
        scan_node,        
    ])