import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

matplotlib.use('TkAgg')  # Ensure correct backend

class LidarPlot(Node):
    def __init__(self):
        super().__init__('lidar_plot')
        self.subscription = self.create_subscription(
            LaserScan,
            '/Fscan',
            self.scan_callback,
            10
        )
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Enable interactive mode
        self.plot_initialized = False
        self.bars = None

    def scan_callback(self, msg):
        angles_rad = np.arange(msg.angle_min+np.pi, msg.angle_max+np.pi, msg.angle_increment)
        ranges = np.array(self.transform_values(msg.ranges))
        
        # Convert angles from radians to degrees
        angles_deg = np.degrees(angles_rad)        

        self.update_plot(angles_deg, ranges)

    def update_plot(self, angles_deg, ranges):
        if not self.plot_initialized:
            self.bars = self.ax.bar(angles_deg, ranges, width=1)  # Adjust width as needed
            self.ax.set_xlim(np.min(angles_deg), np.max(angles_deg))
            self.ax.set_ylim(0, np.max(ranges) + 1)
            self.ax.set_xlabel('Angle (degrees)')
            self.ax.set_ylabel('Range (meters)')
            self.plot_initialized = True
        else:
            for bar, range_value in zip(self.bars, ranges):
                bar.set_height(range_value)
                if range_value == 15.0:
                    bar.set_color('cyan')
                else:
                    bar.set_color('blue')

        self.ax.figure.canvas.draw()
        self.ax.figure.canvas.flush_events()
    
    def transform_values(self, data : LaserScan.ranges):
        n= len(data)
        transformed_values= data[180:]+data[:180]
        return transformed_values

def main(args=None):
    rclpy.init(args=args)
    node = LidarPlot()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.01)  # Allow the GUI to update
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()