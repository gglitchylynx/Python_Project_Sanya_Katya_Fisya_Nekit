#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarLogger(Node):
    def __init__(self):
        super().__init__('lidar_logger')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.counter = 0
        self.get_logger().info("LiDAR logger started")

    def lidar_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid = ranges[np.isfinite(ranges) & (ranges > 0.01)]
        
        if len(valid) == 0:
            return
        
        min_dist = float(np.min(valid))
        max_dist = float(np.max(valid))
        
        if self.counter % 10 == 0:
            self.get_logger().info(f"Min: {min_dist:.2f}m | Max: {max_dist:.2f}m")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = LidarLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()