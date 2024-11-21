import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry , Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped , Twist
from rclpy.qos import QoSProfile

import time
import numpy as np
import csv

import math

def quaternion_to_euler(x, y, z, w):
    # Normalizar el cuaternión    
    norm = math.sqrt(x * x + y * y + z * z + w * w)     
    x /= norm     
    y /= norm     
    z /= norm     
    w /= norm
    # Cálculo del ángulo de rotación en el eje X (Roll)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    # Cálculo del ángulo de rotación en el eje Y (Pitch)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2  # Limitamos t2 para evitar errores numéricos
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    # Cálculo del ángulo de rotación en el eje Z (Yaw)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z

def smooth_angle(angle, prev_angle):         
    if angle >= math.pi/1.5 or angle <= -math.pi/1.5:               
        angle = prev_angle 
    else:
        pass
    return angle

def remap(val,x_min, x_max,y_min,y_max):
    return y_min + ((val-x_min)*(y_max-y_min))/(x_max-x_min)

class NavControlNode(Node):
    def __init__(self):
        super().__init__('nav_control_alpha')
        self.get_logger().info('nav_control_alpha Started')

        #self.odom=self.create_subscription(Odometry,'/alpha/odom',self.odom_callback(),10)
        self.odom_sub=self.create_subscription(Odometry, '/alpha/odom',self.odom_callback,10)

        self.cmd_pub=self.create_publisher(Twist,'/alpha/cmd_vel_nav',10)
        self.orientation_prev=0.0
        
        self.path_x = []
        self.path_y = []

        with open("/home/vekkaz/mobile_robotics_ws/src/nav_2024_alpha/Path/trayectoria.csv", 'r') as csvfile:
            reader=csv.reader(csvfile)
            next(reader)
            for row in reader:
                self.path_x.append(float(row[0])) 
                self.path_y.append(float(row[1])) 
        
        self.odom = Odometry()
        self.previous_error_angle=0
        self.time = 0.0
        self.prev_time = time.time()
        self.orientation= 0
        self.prev_angle = 0
        self.i=0
        self.dt = 0.0000000000001
        self.linear_vel = 0.2

        self.move = True

    def odom_callback(self, odom:Odometry):
        self.time = time.time()
        self.dt = (self.time - self.prev_time)*10e-6
        self.prev_time = self.time
        self.odom = odom
        if self.odom.pose.pose.orientation.x==0.0:
            self.orientation= self.orientation_prev
        else:
            _, _, self.orientation=quaternion_to_euler(self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w)
        self.control()
        self.orientation_prev=self.orientation
                
    def limit (self, data, limit:float):
        if data >= limit:
            return limit
        elif data <= -limit:
            return -limit
        else:
            return data

    def control(self):

        print("CONTROLANDOOOOOOOOOOOOOO!!!") 
        new_vel=Twist()   
        next_point = 1
    
        
        kp_angle= 1.45
        kd_angle = 0.005
        kp_linear = 0.3

        if self.i >= len(self.path_x):
                self.i=len(self.path_x)-1
                self.move = False
        
        print(f"punto a seguir: {self.i+1} \nCoordenadas X:{self.path_x[self.i]}\nCoordenadas Y:{self.path_y[self.i]}\n")
        print(f"Orientacion actual:{math.degrees(self.orientation)}")

        error_angle = self.orientation - np.arctan2(self.path_y[self.i]-self.odom.pose.pose.position.y,self.path_x[self.i]-self.odom.pose.pose.position.x)
        error_angle=-error_angle
        error_angle = smooth_angle(error_angle,self.previous_error_angle)
        error_angle=self.limit(error_angle,np.radians(45.0))

        print(f"----------------------------------------------\nCoordenada X actual: {self.odom.pose.pose.position.x}\nCoordenada Y actual: {self.odom.pose.pose.position.y}")
        print(len(self.path_x))
        error_dist = np.abs(np.sqrt(np.power(self.path_y[self.i],2) + np.power(self.path_x[self.i],2)) - np.sqrt(np.power(self.odom.pose.pose.position.x,2) + np.power(self.odom.pose.pose.position.y,2)))

        steering_angle = self.limit(np.arctan2((2*0.23*np.sin(np.pi/2-error_angle)),error_dist),np.radians(35))
        steering_angle = remap(steering_angle,-np.radians(35),np.radians(35),-1,1)

        error_der_angle = (error_angle - self.previous_error_angle)/self.dt
        self.previous_error_angle=error_angle
        print(error_angle)
        print(error_dist)

        if(abs(error_dist) > 0.35 and self.move):        
                     
            new_vel.angular.z = kp_angle*error_angle 
            #new_vel.angular.z = steering_angle 
              
            if error_angle != 0.0:       
                new_vel.linear.x = np.abs(1/error_angle)*kp_linear
            else:
                new_vel.linear.x = np.abs(1/0.00001)*kp_linear

            new_vel.angular.z=self.limit(new_vel.angular.z,1.0)
            #new_vel.linear.x=self.limit(new_vel.linear.x,0.55)
            new_vel.linear.x=0.55
            self.cmd_pub.publish(new_vel) 

        elif not self.move:  
            new_vel.angular.z = 0.0
            new_vel.linear.x = 0.0
            print("Trayectoria completada!!!")

            self.cmd_pub.publish(new_vel)

        else:
            self.i+=next_point   
                       


def main(args=None):
    rclpy.init(args=args)
    node = NavControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()