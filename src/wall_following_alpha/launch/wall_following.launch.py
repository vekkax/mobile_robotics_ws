
# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='wall_following_alpha' #<--- CHANGE ME
    
    dist_finder = Node(
                    package=package_name, 
                    executable='dist_finder_alpha',
    )

    control = Node(
                    package=package_name, 
                    executable='control_alpha',
    )  


    # Launch them all!
    return LaunchDescription([
        dist_finder,
        control,
    ])