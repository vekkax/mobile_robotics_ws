import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import numpy as np
import matplotlib.pyplot as plt

class LidarPlot(Node):
    def __init__(self):
        super().__init__('lidar_plot')

        # Subscriptions
        self.subscription = self.create_subscription(LaserScan, '/alpha/Fscan', self.scan_callback, 1)
        self.gap = self.create_subscription(Int32MultiArray, "/scan/gap", self.gap_callback, 2)
        self.ftg_subscription = self.create_subscription(Float32MultiArray, "/scan/ranges", self.ftg_ranges_callback, 2)

        # Plot setup
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 8))
        plt.ion()  # Enable interactive mode
        self.plot_initialized = False
        self.bars = None
        self.ftg_bars = None
        self.ftg_ranges = []
        self.specific_indexes = []

    def ftg_ranges_callback(self, msg: Float32MultiArray):
        self.ftg_ranges = msg.data
        self.update_ftg_plot(self.ftg_ranges)

    def gap_callback(self, msg: Int32MultiArray):
        max_gap, max_gap_end_index = msg.data
        self.specific_indexes = range(max_gap_end_index - max_gap + 110, max_gap_end_index + 110)
        self.specific_indexes_ftg = range(max_gap_end_index - max_gap, max_gap_end_index)

    def scan_callback(self, msg: LaserScan):
        angles_rad = np.arange(msg.angle_min + np.pi, msg.angle_max + np.pi, msg.angle_increment)
        ranges = np.array(self.transform_values(msg.ranges))
        
        # Convert angles from radians to degrees
        angles_deg = np.degrees(angles_rad)

        self.update_scan_plot(angles_deg, ranges)

    def update_scan_plot(self, angles_deg, ranges):
        if not self.plot_initialized:
            # Top subplot for LaserScan
            self.bars = self.ax1.bar(angles_deg, ranges, width=1)  # Adjust width as needed
            self.ax1.set_xlim(np.min(angles_deg), np.max(angles_deg))
            self.ax1.set_ylim(0, np.max(ranges) + 1)
            self.ax1.set_xlabel('Angle (degrees)')
            self.ax1.set_ylabel('Range (meters)')
            self.ax1.set_title('LaserScan Data')
            
            # Bottom subplot for ftg_ranges
            # Initialize ftg_bars with dummy data to ensure it gets plotted
            if len(self.ftg_ranges) > 0:
                self.ftg_bars = self.ax2.bar(np.arange(len(self.ftg_ranges)), self.ftg_ranges, width=1, color='blue')
            else:
                # Initialize with zeros if ftg_ranges is empty
                self.ftg_bars = self.ax2.bar(np.arange(10), np.zeros(10), width=1, color='blue')  # Change 10 to desired default size
            self.ax2.set_ylim(0, 1)  # Set a default y-limit
            self.ax2.set_xlabel('Index')
            self.ax2.set_ylabel('Value')
            self.ax2.set_title('FTG Ranges')
            
            self.plot_initialized = True
        else:
            for bar, range_value in zip(self.bars, ranges):
                bar.set_height(range_value)
                if range_value == 15.0:
                    bar.set_color('cyan')
                else:
                    bar.set_color('blue')
            
            for i, bar in enumerate(self.bars):
                if i in self.specific_indexes:
                    bar.set_color('red')  # Set the color of the bar at specific indexes to red
        
        self.ax1.figure.canvas.draw()

    def update_ftg_plot(self, ftg_ranges):
        if self.plot_initialized:
            # Update the bottom subplot with new ftg_ranges data
            # Remove old bars and create new ones to ensure colors are applied
            [bar.remove() for bar in self.ftg_bars]
            self.ftg_bars = self.ax2.bar(np.arange(len(ftg_ranges)), ftg_ranges, width=1, color='blue')
            self.ax2.set_ylim(0, np.max(ftg_ranges) + 1)
            for i, bar in enumerate(self.ftg_bars):
                if i in self.specific_indexes_ftg:
                    bar.set_color('red')
        
        self.ax2.figure.canvas.draw()

    def transform_values(self, data):
        n = len(data)
        transformed_values = data[180:] + data[:180]
        return transformed_values

def main(args=None):
    rclpy.init(args=args)
    node = LidarPlot()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            plt.pause(0.005)  # Allow the GUI to update
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
