import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped , Twist
from rclpy.qos import QoSProfile
import csv
import time
import numpy as np
from sys import exit

class NavRozo(Node):
    def __init__(self):
        super().__init__('nav_rozo_alpha')
        self.get_logger().info('nav_rozo_alpha Started')

        self.odom_sub=self.create_subscription(Odometry, '/alpha/odom',self.odom_callback,10)
        self.timer= self.create_timer(0.3,self.timer_callback)

        self.path_x = []
        self.path_y = []
        self.odom = Odometry()
        self.previous_error_angle=0
        self.time = 0.0
        self.prev_time = time.time()
        self.orientation= 0
        self.flag=False
        self.completion=False
        self.i=9
        self.dt = 0.0000000000001
        self.linear_vel = 0.2

        self.move = True
        
    def odom_callback(self,msg:Odometry):
        self.odom=msg
        x=self.odom.pose.pose.position.x
        y=self.odom.pose.pose.position.y
        if (1.3<x<1.8)and(1.0<y<1.1) and not(self.completion):
            print("pasa por meta") 
            print(self.flag)
            if not(self.flag):
                self.flag = True
                time.sleep(1)
            else:
                self.completion=True
                self.on_close()
            
    def on_close(self):   
        self.path_x.append(self.path_x[0])
        self.path_y.append(self.path_y[0])
        # opens or creates a csv file to write on it
        with open("/home/vekkaz/mobile_robotics_ws/src/nav_2024_alpha/Path/trayectoria.csv", "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            # creates the header of each column
            writer.writerow(
                ["X", "Y"]
            )
            # writes down all the data gathered so far
            for x, y in zip(self.path_x, self.path_y):
                writer.writerow([x,y])
            
            exit()
    
    def timer_callback(self):
        if self.flag and not(self.completion):
            self.path_x.append(self.odom.pose.pose.position.x)
            self.path_y.append(self.odom.pose.pose.position.y)
            print(f"X:{self.path_x[-1]} - Y:{self.path_y[-1]}")
        
                
        

def main(args=None):
    while True:
        rclpy.init(args=args)
        node = NavRozo()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()