
#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

import math


class Control(Node):  # Redefine node class
    def __init__(self):
        super().__init__("control_alpha")  # Redefine node name

        self.error_sub = self.create_subscription(Float32MultiArray, "/Error",self.error_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel",self.vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav",10)
        self.AEB_sub= self.create_subscription(Bool, "/AEB", self.aeb_callback,10)

        self.current_vel = Twist()
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.time = 0.0
        self.iteration = 0
        self.prev_vel = 0.0
        self.aeb_data = False

    def vel_callback(self, data : Twist):
        self.current_vel = data

    def aeb_callback(self, data:Bool):
        self.aeb_data=data.data

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
        kd = 0.50
        kp= 0.45   
        
        if not(self.aeb_data):
            if (math.isnan(self.current_error) or math.isinf(self.current_error))  :
                    
                    new_vel.linear.x = 3.8
                    new_vel.angular.z = self.prev_vel
            
            else:
            
                if self.iteration >= 5:
                    new_vel.angular.z = self.current_error*kp + kd*self.derivative_error #+ self.integral_error*ki
                    if math.isinf(new_vel.angular.z) or math.isnan(new_vel.angular.z):
                        new_vel.angular.z=self.prev_vel
                    self.prev_vel=-new_vel.angular.z
                    

                    if self.current_vel.linear.x < 3.8:                
                        new_vel.linear.x = self.iteration*0.14
                    else:
                        new_vel.linear.x = 3.8

                else:
                    new_vel.angular.z = self.alpha*5.0
                    new_vel.linear.x = 0.7
            
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
