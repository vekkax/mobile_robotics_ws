#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import math
import sys


class DistFinder(Node):  # Redefine node class
    def __init__(self):
        super().__init__("dist_finder_alpha")  # Redefine node name

        self.pose_subs = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.error_pub = self.create_publisher(Float32MultiArray, "/Error", 10)
        self.cmd_vel=self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.Trajd = 0.9
        self.CD = float()
        self.AB = float()

        self.vel=Twist()
        self.vel.linear.x=1.0
        self.dt= 0.0
        self.prev_time= 0

    def scan_callback(self, data:LaserScan):
        self.get_range(data, 70)
        
        current_time = data.header.stamp.sec + data.header.stamp.nanosec * math.pow(10, -9)
        self.dt=current_time - self.prev_time
        error = [self.Trajd - self.CD, self.dt, self.alpha]
        
        self.prev_time = current_time

        if sys.argv and sys.argv[1] == "y":

            self.cmd_vel.publish(self.vel)
        else:
            pass
        
        self.error_pub.publish(Float32MultiArray(data=error))

        

    def get_range(self, data, theta):
        AC = self.dt*self.vel.linear.x
        a = data.ranges[89]
        b = data.ranges[89 + theta]
        self.alpha = math.atan(
            (a * math.cos(math.radians(theta)) - b)
            / (a * math.sin(math.radians(theta)))
        )
        self.AB = b * math.cos(self.alpha)
        self.CD = self.AB + AC * math.sin(self.alpha)


def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = DistFinder()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
