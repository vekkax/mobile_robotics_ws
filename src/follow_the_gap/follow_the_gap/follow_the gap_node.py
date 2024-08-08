#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

import math

class Control(Node):  # Redefine node class
    def __init__(self):
        super().__init__("follow_the_gap_node")  # Redefine node name
        
        self.pose_subs = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        
        self.iteration = 0
        self.ranges = []
        self.min_index = 0
        self.radius = 0.25
        self.threshold = 4.0
        self.dist = 0.0   

        self.max_gap_end_index= 0
        self.max_gap = 0     

        self.x = [] 
        self.y = []       

    def scan_callback(self, data : LaserScan):
        self.iteration += 1
        if self.iteration >= 5:
            self.iteration = 0
            self.ranges = data.ranges[99:259]
            self.min_index = self.ranges.index(min(self.ranges))

            for i in range(len(self.ranges)):
                self.x[i] = self.ranges[i]*math.cos(math.radians(100 + i))
                self.y[i] = self.ranges[i]*math.sin(math.radians(100 + i))

            for i in range(len(self.ranges)):
                if self.isInside(self.x[self.min_index],self.y[self.min_index],self.radius,self.x[i],self.y[i]) or self.ranges[i] < self.threshold:
                    self.ranges[i] = 0.0

            self.max_gap, self.max_gap_end_index = self.find_best_subsection(self.ranges)
            self.control(self.max_gap_end_index + 99)
                    


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

    def control(self, gap_index):
        pass

    def isInside(self, circle_x, circle_y, rad, x, y):
        if ((x - circle_x) * (x - circle_x) + (y - circle_y) * (y - circle_y) <= rad * rad):
            return True
        else:
            return False               

        
    def error_callback(self, data : Float32MultiArray):
        self.iteration += 1
        self.current_error = data.data[0]
        self.dt = data.data[1]
        self.alpha = data.data[2]
        new_vel = Twist()
        
        self.derivative_error= (self.current_error - self.previous_error)/self.dt
        self.previous_error=self.current_error
        self.integral_error += self.current_error / self.dt
        self.time += self.dt/100

        ki = 0.0001
        kd = 2.5
        kp= 2.0   
        
        if not(self.aeb_data):
            if (math.isnan(self.current_error) or math.isinf(self.current_error)):                    
                    new_vel.linear.x = 1.0
                    new_vel.angular.z = self.prev_vel            
            else:
                if (self.right_ray >= self.desire_dist*1.5 and self.left_ray >= self.desire_dist*1.5):
                    new_vel.linear.x = self.current_vel.linear.x 
                    new_vel.angular.z = 2.0
                else:            
                    if self.iteration >= 2:
                        new_vel.angular.z = self.current_error*kp + kd*self.derivative_error #+ self.integral_error*ki
                        if math.isinf(new_vel.angular.z) or math.isnan(new_vel.angular.z):
                            new_vel.angular.z=self.prev_vel
                        self.prev_vel=-new_vel.angular.z                        

                        if self.current_vel.linear.x < 1.0:                
                            new_vel.linear.x = self.iteration*0.14
                        else:
                            new_vel.linear.x = 1.0
                            
                    else:
                        new_vel.angular.z = self.alpha*0.50
                        new_vel.linear.x = 0.3
            
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
