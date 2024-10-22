#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import statistics as sts

class ScanNodeG01(Node):
    def __init__(self):
        super().__init__("scan_node")

        # Subscribing and publishing topics
        self.scan_subs = self.create_subscription(LaserScan, '/alpha/scan', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, "/alpha/Fscan", 10)

        self.msg = LaserScan()

    def scan_callback(self, data: LaserScan):
        self.msg = data
        ranges = self.msg.ranges

        # Handle NaN and infinite values
        for i in range(len(ranges)):
            if math.isnan(ranges[i]):
                ranges[i] = 0.1  # Replace NaN with small value (e.g., 0.1)
            elif ranges[i] == math.inf:
                ranges[i] = 15.0  # Replace infinite values with maximum range

        # Apply median filtering to smooth the values
        for i in range(len(ranges)):
            if i < 5:  # For the first few rays
                self.msg.ranges[i] = self.median_filter(ranges, i, window_size=5)
            else:  # For other rays, apply the measurement function or median filter
                self.msg.ranges[i] = self.median_filter(ranges, i, window_size=5)

        # Publish the filtered message
        self.scan_pub.publish(self.msg)

    def median_filter(self, ranges, index, window_size=10):
        """
        Applies a median filter to the ranges at the given index.
        
        Args:
            ranges (list): List of LiDAR range values.
            index (int): The current index of the range value to filter.
            window_size (int): The size of the window for the median filter (default is 5).
            
        Returns:
            float: The median value of the window.
        """
        # Ensure indices don't go out of bounds
        start = max(0, index - window_size // 2)
        end = min(len(ranges), index + window_size // 2 + 1)

        # Filter out invalid values (e.g., 15.0 as max range)
        valid_rays = [r for r in ranges[start:end] if r != 15.0]
        
        # If all values are invalid, return the default maximum range
        if len(valid_rays) == 0:
            return 15.0

        # Return the median of the valid values
        return sts.median(valid_rays)

def main(args=None):
    rclpy.init(args=args)

    # Create node object
    node = ScanNodeG01()

    # Keep the node running
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
