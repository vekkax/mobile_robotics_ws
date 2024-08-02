#!/usr/bin/env python3

#libreria cliente de python
import json
import os
import rclpy
from math import pow
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanNodeG01(Node): # Redefine node class
    def __init__(self):
        

        directory = '/home/vekkaz/mobile_robotics_ws/src/scan_pkg//data'
        filename = 'scan_log.json'
        self.filepath = os.path.join(directory, filename)
        os.makedirs(directory, exist_ok=True)
        if os.path.exists(self.filepath):
            os.remove(self.filepath)


        super().__init__("scan_node") # Redefine node name

        self.pose_subs = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)

        # create a timer function to send msg
        timer_period = 0.75 # in [s]
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.msg = LaserScan()
        self.data_dict= {"Scan_time":[],"Ranges":[]}
        
            
    


    def timer_callback(self):
        
        self.data_dict["Scan_time"].append(self.msg.header.stamp.sec + self.msg.header.stamp.nanosec*pow(10, -9))
        self.data_dict["Ranges"].append(list(self.msg.ranges))

        with open(self.filepath, 'w') as json_file:
            json.dump(self.data_dict, json_file, indent=4)

        print("------------------------------------",
              "\nScan Time:", str(self.data_dict["Scan_time"][-1]),
              "\nRanges:\n",
              str(self.data_dict["Ranges"][-1])
              )

    def scan_callback(self,data):
        self.msg = data
            


def main(args=None):

    #inicializador del nodo
    rclpy.init(args=args)

    #objeto nodo
    node = ScanNodeG01() # object definition (creation)

    rclpy.spin(node)
    

    rclpy.shutdown()

if __name__ == "__main__":
    main()