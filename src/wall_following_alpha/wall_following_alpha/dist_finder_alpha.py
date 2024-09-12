#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32

import statistics as sts
import math

class DistFinder(Node):  # Redefine node class
    def __init__(self):
        super().__init__("dist_finder_alpha")  # Redefine node name

        self.pose_subs = self.create_subscription(LaserScan, "/Fscan", self.scan_callback, 10)
        self.error_pub = self.create_publisher(Float32MultiArray, "/Error", 10)  
        self.rays_pub = self.create_publisher(Float32MultiArray, "/Rays", 10)        
        self.distance_pub = self.create_publisher(Float32, "/Desire_dist", 10)

        self.Trajd = 1.0
        self.CD = float()
        self.AB = float()

        self.vel=Twist()
        self.vel.linear.x=1.0
        self.dt= 0.0
        self.prev_time= 0

    def scan_callback(self, data:LaserScan):
        self.get_range(data, -80)
        
        current_time = data.header.stamp.sec + data.header.stamp.nanosec * math.pow(10, -9)
        self.dt=current_time - self.prev_time
        error = [self.Trajd - self.CD, self.dt, self.alpha]
        
        self.prev_time = current_time        
        
        self.error_pub.publish(Float32MultiArray(data=error))
        self.distance_pub.publish(Float32(data=self.Trajd))

        

    def get_range(self, data, theta):
        AC = self.dt*self.vel.linear.x
        a = self.measurment(data,89) # Cambiar por -90 o 270 si se quiere seguir la pared derecha 
        b = self.measurment(data,89 + theta)
        self.alpha = math.atan(
            (a * math.cos(math.radians(theta)) - b)
            / (a * math.sin(math.radians(theta)))
        )
        self.AB = b * math.cos(self.alpha)
        self.CD = self.AB + AC * math.sin(self.alpha)

    def measurment(self, data, mid):
        rays = data.ranges[mid-5:mid+5]
        self.rays_pub.publish(Float32MultiArray(data=rays))
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

    # inicializador del nodo
    rclpy.init(args=args)

    # objeto nodo
    node = DistFinder()  # object definition (creation)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
