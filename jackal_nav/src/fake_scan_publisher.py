#!/usr/bin/env python3
"""
Fake LiDAR scan publisher for testing navigation without real robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class FakeScanPublisher(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        
        self.get_logger().info("Fake scan publisher started")
    
    def publish_scan(self):
        scan = LaserScan()
        
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Create fake scan data - a room with walls at 5 meters
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = []
        
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            
            # Simple room simulation - walls at distance
            if abs(angle) < math.pi/4:  # Front
                range_val = 5.0 + np.random.normal(0, 0.1)  # 5m with noise
            elif abs(angle - math.pi/2) < math.pi/4 or abs(angle + math.pi/2) < math.pi/4:  # Sides
                range_val = 3.0 + np.random.normal(0, 0.1)  # 3m with noise
            else:  # Back
                range_val = 2.0 + np.random.normal(0, 0.1)  # 2m with noise
            
            # Add some random obstacles
            if np.random.random() < 0.01:  # 1% chance of obstacle
                range_val = np.random.uniform(1.0, 3.0)
            
            ranges.append(max(scan.range_min, min(range_val, scan.range_max)))
        
        scan.ranges = ranges
        scan.intensities = [1.0] * len(ranges)
        
        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    
    node = FakeScanPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 