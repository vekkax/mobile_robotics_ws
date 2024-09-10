#!/usr/bin/env python3

#libreria cliente de python
import json
import os
import rclpy
from math import pow
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Float32MultiArray
import statistics as sts

class ScanNodeG01(Node): # Redefine node class
    def __init__(self):

        super().__init__("scan_node") # Redefine node name

        self.scan_subs = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.scan_pub = self.create_publisher(LaserScan, "/Fscan", 10) 

        
        self.msg = LaserScan()   

    def scan_callback(self,data:LaserScan):
        self.msg = data
        ranges = self.msg.ranges
        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                ranges[i] = 0.25
            elif ranges[i]==math.inf:
                ranges[i] = 15.0

        self.msg.ranges = ranges
        for i in range(len(ranges)):
            if i == 0 or i == 1 or i == 2 or i == 3 or i == 4:
                self.msg.ranges[i] = sts.mean(self.msg.ranges[-3:-1])
            else: 
                self.msg.ranges[i] = self.measurment(self.msg,i)

        self.scan_pub.publish(self.msg)

    def measurment(self, data, mid):
        rays = data.ranges[mid-5:mid+5]        
        if all(ray==15.0 for ray in rays):  
            return 15.0
        else:  
            delete = []
            for i in range(len(rays)):
                if rays[i] == 15.0:
                    delete.append(i)
            for ele in sorted(delete, reverse = True): 
                del rays[ele]

            return sts.mean(rays)
            


def main(args=None):

    #inicializador del nodo
    rclpy.init(args=args)

    #objeto nodo
    node = ScanNodeG01() # object definition (creation)

    rclpy.spin(node)
    

    rclpy.shutdown()

if __name__ == "__main__":
    main()