#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import statistics as sts
import time

import math
import sys


class TTC(Node):  # Redefine node class
    def __init__(self):
        super().__init__("time_to_collision_node")  # Redefine node name

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.pose_subs = self.create_subscription(LaserScan, "/Fscan", self.scan_callback, 10)

        self.aeb_pub = self.create_publisher(Twist, "/cmd_vel_aeb", 10)
        self.aeb_act = self.create_publisher(Bool, "/AEB", 10)
        self.TTC = self.create_publisher(Float32, "/TTC", 10)

        
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.cmd_break= Twist()
        self.cmd_break.linear.x = 0.0
        self.cmd_break.angular.z = 0.0

        self.aeb_data = Bool()
        self.aeb_data.data = False 

        self.vel = float()
        self.dist = float()
        
    def timer_callback(self):        
        if self.vel > 0.0 and self.dist != 0.0 and not(self.aeb_data.data):            
            r_d = self.vel *math.cos(math.radians(1))
            ttc= self.dist/r_d
            self.TTC.publish(Float32(data=ttc))
            if ttc <= 1.2:
                self.cmd_break.linear.x = -1.0
                self.aeb_pub.publish(self.cmd_break)
                time.sleep(0.8)
                self.aeb_data.data=True
                self.aeb_act.publish(self.aeb_data)
            else:
                self.aeb_data.data=False
            self.aeb_act.publish(self.aeb_data)   
        elif self.aeb_data.data:
            self.aeb_data.data=True
            self.cmd_break.linear.x = 0.0
            self.aeb_pub.publish(self.cmd_break)
            if self.dist > 1.0:
                self.aeb_data.data=False
            self.aeb_act.publish(self.aeb_data) 

    def cmd_vel_callback(self, data: Twist):
        self.vel=data.linear.x

    def scan_callback(self, data: LaserScan):
        self.dist = data.ranges[-2:] +data.ranges[:2]
        self.dist = sts.mean(self.dist)
    

def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = TTC()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
