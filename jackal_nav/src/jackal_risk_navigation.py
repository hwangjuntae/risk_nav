#!/usr/bin/env python3
"""
Jackal Risk Navigation ROS2 Node
Simplified risk assessment for Jackal robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import json
import time
import math

# cv_bridge import
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    print("cv_bridge package not found. Please install ros-humble-cv-bridge.")
    CV_BRIDGE_AVAILABLE = False

class JackalRiskNavigationNode(Node):
    def __init__(self):
        super().__init__('jackal_risk_navigation')
        
        # cv_bridge 초기화
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().error("cv_bridge is not available.")
        
        # Publishers
        self.risk_map_publisher = self.create_publisher(
            OccupancyGrid, 
            '/risk_map', 
            10
        )
        
        self.risk_image_publisher = self.create_publisher(
            Image, 
            '/risk_assessment/image', 
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_risk',
            10
        )
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Variables
        self.latest_scan = None
        self.latest_image = None
        self.latest_cmd_vel = None
        
        # Risk assessment parameters
        self.min_safe_distance = 1.0  # meters
        self.warning_distance = 2.0   # meters
        self.max_safe_speed = 1.0     # m/s
        
        # Control parameters
        self.target_fps = 10.0
        self.frame_interval = 1.0 / self.target_fps
        self.last_process_time = 0.0
        
        # Timer for risk assessment
        self.timer = self.create_timer(0.1, self.risk_assessment_timer)
        
        self.get_logger().info("Jackal Risk Navigation Node initialized")
    
    def image_callback(self, msg):
        """Camera image callback"""
        try:
            if self.bridge is not None:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def scan_callback(self, msg):
        """LiDAR scan callback"""
        self.latest_scan = msg
    
    def cmd_vel_callback(self, msg):
        """Command velocity callback"""
        self.latest_cmd_vel = msg
    
    def risk_assessment_timer(self):
        """Main risk assessment timer callback"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_process_time < self.frame_interval:
            return
        
        self.last_process_time = current_time
        
        if self.latest_scan is not None:
            self.assess_risk_from_lidar()
        
        if self.latest_image is not None and self.bridge is not None:
            self.assess_risk_from_camera()
    
    def assess_risk_from_lidar(self):
        """Assess risk based on LiDAR data"""
        if self.latest_scan is None:
            return
        
        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        
        # Remove invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]
        valid_ranges = valid_ranges[valid_ranges > scan.range_min]
        valid_ranges = valid_ranges[valid_ranges < scan.range_max]
        
        if len(valid_ranges) == 0:
            return
        
        # Find minimum distance
        min_distance = np.min(valid_ranges)
        
        # Assess risk level
        risk_level = self.calculate_risk_level(min_distance)
        
        # Adjust command velocity based on risk
        if self.latest_cmd_vel is not None:
            modified_cmd_vel = self.modify_cmd_vel(self.latest_cmd_vel, risk_level)
            self.cmd_vel_publisher.publish(modified_cmd_vel)
        
        # Log risk assessment
        if risk_level > 0.7:
            self.get_logger().warn(f"High risk detected! Min distance: {min_distance:.2f}m")
        elif risk_level > 0.4:
            self.get_logger().info(f"Medium risk. Min distance: {min_distance:.2f}m")
    
    def assess_risk_from_camera(self):
        """Simple visual risk assessment"""
        if self.latest_image is None:
            return
        
        # Simple edge detection for obstacle detection
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Count edge pixels as a simple measure of visual complexity/obstacles
        edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])
        
        # Create risk visualization
        risk_image = self.create_risk_visualization(self.latest_image, edge_density)
        
        # Publish risk image
        try:
            risk_msg = self.bridge.cv2_to_imgmsg(risk_image, "bgr8")
            self.risk_image_publisher.publish(risk_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish risk image: {e}")
    
    def calculate_risk_level(self, distance):
        """Calculate risk level based on distance"""
        if distance < self.min_safe_distance:
            return 1.0  # Maximum risk
        elif distance < self.warning_distance:
            # Linear interpolation between warning and safe distance
            return 1.0 - (distance - self.min_safe_distance) / (self.warning_distance - self.min_safe_distance)
        else:
            return 0.0  # No risk
    
    def modify_cmd_vel(self, original_cmd_vel, risk_level):
        """Modify command velocity based on risk level"""
        modified_cmd_vel = Twist()
        
        # Reduce speed based on risk level
        speed_factor = 1.0 - risk_level * 0.8  # Reduce speed by up to 80%
        speed_factor = max(0.1, speed_factor)  # Minimum 10% speed
        
        modified_cmd_vel.linear.x = original_cmd_vel.linear.x * speed_factor
        modified_cmd_vel.linear.y = original_cmd_vel.linear.y * speed_factor
        modified_cmd_vel.linear.z = original_cmd_vel.linear.z * speed_factor
        
        # Keep angular velocity but reduce if high risk
        angular_factor = 1.0 - risk_level * 0.5  # Reduce angular speed by up to 50%
        angular_factor = max(0.2, angular_factor)  # Minimum 20% angular speed
        
        modified_cmd_vel.angular.x = original_cmd_vel.angular.x * angular_factor
        modified_cmd_vel.angular.y = original_cmd_vel.angular.y * angular_factor
        modified_cmd_vel.angular.z = original_cmd_vel.angular.z * angular_factor
        
        return modified_cmd_vel
    
    def create_risk_visualization(self, image, edge_density):
        """Create risk visualization overlay"""
        risk_image = image.copy()
        height, width = image.shape[:2]
        
        # Create risk overlay based on edge density
        risk_color = (0, 0, 255) if edge_density > 0.1 else (0, 255, 0)
        risk_text = f"Risk: {edge_density:.3f}"
        
        # Add text overlay
        cv2.putText(risk_image, risk_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, risk_color, 2)
        
        # Add risk indicator bar
        bar_width = int(width * 0.3)
        bar_height = 20
        bar_x = width - bar_width - 10
        bar_y = 10
        
        # Background bar
        cv2.rectangle(risk_image, (bar_x, bar_y), 
                     (bar_x + bar_width, bar_y + bar_height), (128, 128, 128), -1)
        
        # Risk level bar
        risk_width = int(bar_width * min(edge_density * 10, 1.0))
        cv2.rectangle(risk_image, (bar_x, bar_y), 
                     (bar_x + risk_width, bar_y + bar_height), risk_color, -1)
        
        return risk_image

def main(args=None):
    rclpy.init(args=args)
    
    node = JackalRiskNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 