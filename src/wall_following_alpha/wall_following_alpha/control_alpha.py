
#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import statistics as sts

import math


class Control(Node):  # Redefine node class
    def __init__(self):
        super().__init__("control_alpha")  # Redefine node name

        self.error_sub = self.create_subscription(Float32MultiArray, "/Error",self.error_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel",self.vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav",10)
        self.AEB_sub= self.create_subscription(Bool, "/AEB", self.aeb_callback,10)
        self.pose_subs = self.create_subscription(LaserScan, "/Fscan", self.scan_callback, 10)
        self.desire_dist_sub = self.create_subscription(Float32, "/Desire_dist", self.desire_dist_callback, 10)


        self.current_vel = Twist()
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.time = 0.0
        self.iteration = 0
        self.prev_vel = 0.0
        self.aeb_data = False
        self.desire_dist = float()
        self.right_ray = float()
        self.left_ray = float()
        self.front_ray = float() 
        self.nw_it = 0
        self.speed = 0.70



    def desire_dist_callback(self, data : Float32):
        self.desire_dist = data.data

    def scan_callback(self, data : LaserScan):
        self.right_ray_avg = self.measurment(data,-69)
        self.left_ray_avg = self.measurment(data,69)
        self.front_ray_avg = self.measurment(data,0)
        

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

    def vel_callback(self, data : Twist):
        self.current_vel = data

    def aeb_callback(self, data:Bool):
        self.aeb_data=data.data

    def error_callback(self, data : Float32MultiArray):
       # self.iteration += 1
        self.current_error = -data.data[0]
        self.dt = data.data[1]
        self.alpha = data.data[2]
        new_vel = Twist()
        
        self.derivative_error= (self.current_error - self.previous_error)/self.dt
        self.previous_error=self.current_error
        self.integral_error += self.current_error / self.dt
        self.time += self.dt/100

        ki = 0.0000001
        kd = 0.8
        kp= 0.3 #0.8       

       # if self.iteration == 1:
        new_vel.angular.z = self.current_error*kp + kd*self.derivative_error  #+ki*self.integral_error
        new_vel.linear.x = self.speed 

        ############ SIMULATION ############    
        #self.iteration = 0       
        #else:
        #    new_vel.angular.z = self.current_vel.angular.z 
        #    new_vel.linear.x = self.current_vel.linear.x     

    #    if not(self.aeb_data):
    #        if (math.isnan(self.current_error) or math.isinf(self.current_error)):                    
    #                new_vel.linear.x = self.speed
    #                new_vel.angular.z = self.prev_vel            
    #        else:
    #            if self.front_ray <= self.desire_dist*2.3:
    #                new_vel.linear.x = self.current_vel.linear.x - self.current_vel.linear.x/6
    #                new_vel.angular.z = 1.0
    #            elif (self.right_ray >= self.desire_dist*1.5 and self.left_ray >= self.desire_dist*1.5):
    #                new_vel.linear.x = self.current_vel.linear.x
    #                new_vel.angular.z = -1.0
    #            else:            
    #                if self.iteration >= 2:
    #                    new_vel.angular.z = self.current_error*kp + kd*self.derivative_error #+ self.integral_error*ki
    #                    if math.isinf(new_vel.angular.z) or math.isnan(new_vel.angular.z):
    #                        new_vel.angular.z=self.prev_vel
    #                    self.prev_vel=-new_vel.angular.z                        

    #                    if self.current_vel.linear.x < self.speed:
    #                        if self.iteration <= 10:
    #                            new_vel.linear.x = self.speed*0.75
    #                        else:
    #                            new_vel.linear.x = self.current_vel.linear.x + self.current_vel.linear.x/8
    #                    else:
    #                        new_vel.linear.x = self.speed

    #                else:
    #                    new_vel.angular.z = self.alpha*0.50
    #                    new_vel.linear.x = 0.3

    ############ REAL CAR ############
        if self.front_ray_avg <= 0.5:
            
            if self.left_ray_avg > self.right_ray_avg:
                new_vel.angular.z = 1.0
            else:
                new_vel.angular.z = -1.0

        elif self.left_ray_avg >= 3.0:            
            new_vel.angular.z = 0.8

        elif self.right_ray_avg >= 3.0 and self.front_ray_avg <= 1.0:
            new_vel.angular.z = -0.8
            
        new_vel.linear.x = self.limit(new_vel.linear.x,1.0)
        new_vel.angular.z = self.limit(new_vel.angular.z,0.7)
        self.cmd_vel_pub.publish(new_vel)

    def limit (self, data, limit:float):
        if data >= limit:
            return limit
        elif data <= -limit:
            return -limit
        else:
            return data

        
  

def main(args=None):

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = Control()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
