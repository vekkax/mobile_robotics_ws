from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory('f112th_sim_2024_alpha'), 'description', 'odom_tf_robot','odom_to_tf.urdf.xacro')

    return LaunchDescription([
        # Start the robot_state_publisher to publish the URDF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='alpha',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        
        # Start the custom node that broadcasts odom -> base_link based on odometry
        Node(
            package='odom_to_tf',
            executable='odom_to_tf_node',
            name='odom_to_tf_node',
            output='screen',
            namespace='alpha'
        )
    ])