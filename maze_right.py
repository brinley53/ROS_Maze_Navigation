#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import time

# Constants
MOVING_SPEED = 0.2
WALL_DISTANCE = 0.3  # Minimum distance from wall (meters) (4 in ~ 0.1 m). take car into account
TURN_SPEED = 1.0  # Speed for turning away from walls
FORWARD_DISTANCE = 0.3  # Distance to move forward (meters)

END_DISTANCE = 0.1 # Minimum distance from blue wall (meters) (4 in ~ 0.1 m)
COLOR_TOLERANCE = 20  # For color detection centering

# BLUE color range
BLUE_LOWER = np.array([73, 155, 94])
BLUE_UPPER = np.array([70, 255, 100])

class MazeNavigator(Node):
    def __init__(self, name):
        super().__init__(name)
        self.turning_around = False
        self.turn_start_time = None

        
        # ROS2 publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)
        self.camera_sub = self.create_subscription(
            Image, 'ascamera/camera_publisher/rgb0/image', 
            self.camera_callback, 1)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan_raw', self.lidar_callback, 1)

        self.bridge = CvBridge()
        self.lidar_data = []
        self.at_end = False

        # Wall-following state
        self.following_wall = False
        self.right_wall_distance = 0.0

        # Main control timer
        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        if self.at_end or not self.lidar_data:
            return

        def safe_min(ranges):
            clean = [r for r in ranges if np.isfinite(r) and r > 0.05]
            return min(clean) if clean else float('inf')

        # Get LiDAR readings for right, front, and left
        right_dist = safe_min(self.lidar_data[270:300])  # Right side (90 deg from front)
        front_dist = safe_min(self.lidar_data[0:30] + self.lidar_data[330:360])  # Front
        left_dist = safe_min(self.lidar_data[60:90])     # Left side

        self.get_logger().info(f"Distances - Front: {front_dist:.2f}, Left: {left_dist:.2f}, Right: {right_dist:.2f}")

        twist = Twist()

        if self.turning_around:
            # Keep turning for 1.5 seconds
            if time.time() - self.turn_start_time < 1.5:
                twist.angular.z = TURN_SPEED
            else:
                self.turning_around = False
                self.get_logger().info("Completed 180 turn")
            self.cmd_vel_pub.publish(twist)
            return

        if right_dist > WALL_DISTANCE:
            twist.angular.z = -TURN_SPEED
            self.get_logger().info("Right is open - turning right")
        elif front_dist < WALL_DISTANCE and right_dist < WALL_DISTANCE and left_dist > WALL_DISTANCE:
            twist.angular.z = TURN_SPEED
            self.get_logger().info("Right and front blocked - turning left")
        elif front_dist < WALL_DISTANCE and right_dist < WALL_DISTANCE and left_dist < WALL_DISTANCE:
            twist.angular.z = TURN_SPEED
            self.turning_around = True
            self.turn_start_time = time.time()
            twist.angular.z = TURN_SPEED
        else:
            twist.linear.x = MOVING_SPEED
            self.get_logger().info("Path clear - moving forward")

        self.cmd_vel_pub.publish(twist)


    def lidar_callback(self, data):
        """Process LiDAR data for wall detection"""
        if self.at_end:
            return
        self.lidar_data = data.ranges

    def camera_callback(self, data):
        """Detect BLUE room and stop"""
        if self.at_end:
            return
            
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        blurred = cv2.GaussianBlur(lab_frame, (5, 5), 0)
        
        BLUE_mask = cv2.inRange(blurred, BLUE_LOWER, BLUE_UPPER)
        contours, _ = cv2.findContours(BLUE_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                if self.lidar_data and min(self.lidar_data[:30] + self.lidar_data[330:]) < END_DISTANCE:
                    self.at_end = True
                    self.cmd_vel_pub.publish(Twist())  # Stop
                    self.get_logger().info("BLUE ROOM REACHED")

def main(args=None):
    rclpy.init(args=args)
    navigator = MazeNavigator('maze_navigator')
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Keyboard Interrupt (Ctrl+C): Stopping node.")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()