#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

import math
import sys


class DistFinder(Node):  # Redefine node class
    def __init__(self):
        super().__init__("dist_finder_alpha")  # Redefine node name

        self.pose_subs = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.error_pub = self.create_publisher(Point, "/Error", 10)
        self.cmd_vel=self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.Trajd = 1.0
        self.CD = float()

        self.vel=Twist()
        self.vel.linear.x=0.8

    def scan_callback(self, data):
        self.get_range(data, 20)
        error = Point()
        error.y = self.Trajd - self.CD
        if sys.argv[1] == "y":
            self.cmd_vel.publish(self.vel)
        elif sys.argv[1]== "n":
            pass

        self.error_pub.publish(error)

    def get_range(self, data, theta):
        AC = 1.0
        a = data.ranges[89]
        b = data.ranges[89 + theta]
        alpha = math.atan(
            (a * math.cos(math.radians(theta)) - b)
            / (a * math.sin(math.radians(theta)))
        )
        AB = b * math.cos(alpha)
        self.CD = AB + AC * math.sin(alpha)


def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = DistFinder()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
