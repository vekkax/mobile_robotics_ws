#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import math

class FTG(Node):  # Redefine node class
    def __init__(self):
        super().__init__("follow_the_gap_node")  # Redefine node name
        
        self.pose_subs = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.vel_subs = self.create_subscription(Twist, "/cmd_vel", self.vel_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav",10)
        
        self.iteration = 0
        self.ranges = []
        self.min_index = 0
        self.radius = 0.25
        self.threshold = 2.7
        self.colission_threshold = 0.5
        self.dist = 0.0 
        self.data = []  

        self.linear_speed = 2.0
        self.angular_speed = 1.0

        self.max_gap_end_index= 0
        self.max_gap = 0     

        self.x = [] 
        self.y = []    

        self.previous_error = 0.0
        self.dt= 0.1

        self.velocity = Twist()


    def vel_callback(self, data : Twist):
        self.velocity = data

    def scan_callback(self, data : LaserScan):
        self.data = data.ranges
        self.iteration += 1
        if self.iteration <= 5:
            val = 135
        else:
            val = 30
        min_range= val - 1
        max_range= 360 - 1 - min_range

        if not(self.iteration % 1):
            self.iteration = 0
            self.ranges = [x - self.radius for x in data.ranges[min_range:max_range]]
            self.min_index = self.ranges.index(min(self.ranges))

            for i in range(len(self.ranges)):
                if math.isinf(self.ranges[i]):
                    self.ranges[i] = 12.0
                self.x.append(self.ranges[i]*math.cos(math.radians(min_range + 1 + i)))
                self.y.append(self.ranges[i]*math.sin(math.radians(min_range + 1 + i)))

            for i in range(len(self.ranges)):
                if self.isInside(self.x[self.min_index],self.y[self.min_index],self.radius,self.x[i],self.y[i]) or self.ranges[i] < self.threshold:
                    self.ranges[i] = 0.0
            
            self.max_gap, self.max_gap_end_index = self.find_best_subsection(self.ranges)
            self.error = (self.max_gap_end_index-self.max_gap/2) + min_range - 180
            #print(error)
            self.control(self.error)


    def find_best_subsection(self, arr):
        max_sum = 0
        max_width = 0
        max_end_index = 0
        current_sum = 0
        current_start = 0
        
        for i, value in enumerate(arr):
            if value != 0:
                if current_sum == 0:
                    current_start = i
                current_sum += value
            else:
                if current_sum > max_sum or (current_sum == max_sum and (i - current_start) > max_width):
                    max_sum = current_sum
                    max_width = i - current_start
                    max_end_index = i - 1
                current_sum = 0        
        
        if current_sum > max_sum or (current_sum == max_sum and (len(arr) - current_start) > max_width):
            max_sum = current_sum
            max_width = len(arr) - current_start
            max_end_index = len(arr) - 1

        return max_width, max_end_index

        

    def control(self, error): 
            
            kp=0.03
            kd = 0.003
            
            error_d = (self.error - self.previous_error)/self.dt
            self.previous_error=self.error
            
            new_vel=Twist()

            threshold = 2.3

            if all(value < threshold for value in self.data[160:200]):
                if all(value < threshold*(1/3) for value in self.data[160:200]):             
                    new_vel.linear.x = self.velocity.linear.x*0.3
                else:
                    new_vel.linear.x = self.velocity.linear.x*0.7                
                if max(self.data[44:89]) > max(self.data[269:314]):
                    new_vel.angular.z = -3.0      
                else:
                    new_vel.angular.z = 3.0
            elif self.data[135] < self.colission_threshold:
                new_vel.linear.x = self.linear_speed*0.75
                new_vel.angular.z = self.angular_speed
            elif self.data[225] < self.colission_threshold:
                new_vel.linear.x = self.linear_speed*0.75
                new_vel.angular.z = -self.angular_speed
            else:
                if self.velocity.linear.x < self.linear_speed:
                    new_vel.linear.x = self.velocity.linear.x + 0.4
                else:
                    new_vel.linear.x = self.linear_speed
                new_vel.angular.z = kp*error + kd*error_d


            self.cmd_pub.publish(new_vel)

        


    def isInside(self, circle_x, circle_y, rad, x, y):
        if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad):
            return True
        else:
            return False               


def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = FTG()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
