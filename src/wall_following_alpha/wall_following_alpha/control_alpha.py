
#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import math


class Control(Node):  # Redefine node class
    def __init__(self):
        super().__init__("control_alpha")  # Redefine node name

        self.error_sub = self.create_subscription(Float32MultiArray, "/Error",self.error_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel",self.vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav",10)

        self.current_vel = Twist()
        self.integral_error= 0.0

    def vel_callback(self, data : Twist):
        self.current_vel = data

    def error_callback(self, data : Float32MultiArray):
        self.error = data.data[0]
        self.dt = data.data[1]
        new_vel = Twist()
        
        self.integral_error += self.error * self.dt

        ki = 0.1
 
        kp= 0.1

        new_vel.angular.z = self.current_vel.angular.z + self.error*kp  + self.integral_error*ki
        new_vel.linear.x = self.current_vel.linear.x
        self.cmd_vel_pub.publish(new_vel)
  

def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = Control()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
