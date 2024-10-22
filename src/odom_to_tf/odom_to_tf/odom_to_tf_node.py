import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose

class OdomToTFNode(Node):

    def __init__(self):
        super().__init__('odom_to_tf_node')

        # Create a TransformBroadcaster to send the transform from odom to base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the odometry topic
        self.odom_sub = self.create_subscription(Pose,'/robot2/pose',self.odom_callback,10)

        self.timer= self.create_timer(0.1, self.timer_callback)
        self.transform = TransformStamped()
        self.msg=Pose()


    def odom_callback(self, msg: Pose):
        # Create a TransformStamped message to represent the transform
        self.msg=msg
        
    def timer_callback(self):
        # Set the header
        msg=self.msg
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = 'odom'  # Fixed frame (parent)
        self.transform.child_frame_id = 'base_link'  # Moving frame (child)

        # Set the translation (position from the odometry message)
        self.transform.transform.translation.x = msg.position.x
        self.transform.transform.translation.y = msg.position.y
        self.transform.transform.translation.z = msg.position.z

        # Set the rotation (orientation from the odometry message)
        self.transform.transform.rotation.x = msg.orientation.x
        self.transform.transform.rotation.y = msg.orientation.y
        self.transform.transform.rotation.z = msg.orientation.z
        self.transform.transform.rotation.w = msg.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(self.transform)


def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    node = OdomToTFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
