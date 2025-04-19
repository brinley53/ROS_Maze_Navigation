#!/usr/bin/env python3

from graph import Graph

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import time

from collections import deque   # BFS uses queue

# Constants
MOVING_SPEED = 0.2
WALL_DISTANCE = 0.3  # Minimum distance from wall (meters) (4 in ~ 0.1 m). take car into account
TURN_SPEED = 1.0  # Speed for turning away from walls
FORWARD_DISTANCE = 0.3  # Distance to move forward (meters)

END_DISTANCE = 0.1 # Minimum distance from blue wall (meters) (4 in ~ 0.1 m)
FORWARD_SPEED = 0.2
COLOR_TOLERANCE = 20  # For color detection centering

TURN_DURATION = 0.785 # maybe 45 degrees?
STEP_DURATION = 0.1 # maybe 5 inches?

#BLUE color range  
BLUE_LOWER = np.array([73, 155, 94])
BLUE_UPPER = np.array([70, 255, 100])

DIRECTIONS = ["forward", 'right', 'backward', 'left']

class MazeNavigator(Node):
    def __init__(self, name):
        super().__init__(name)
        
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
        self.start = True
        
        # Maze navigation variables
        self.graph = Graph()  # Graph representation of maze
        self.current_node = (0, 0)  # Current position in grid
        self.previous_node = (0, 0)
        self.heading = 0  # 0=N, 1=E, 2=S, 3=W
        # self.path = []  # BFS path to follow
        # self.path_index = 0  # Current path position

        self.marked = set()
        self.dfs_stack = [self.current_node]
        self.backtrack_stack = []
        
        # State management
        self.following_wall = False
        self.first_attempt = True
        self.mapping_complete = False

        self.twist = Twist()

        self.moving = False
        self.move_start_time = None
        self.move_target = None

        self.create_timer(0.1, self.dfs_step)

    def lidar_callback(self, data):
        """Process Lidar data for wall detection and navigation"""
        if self.at_end or not self.first_attempt:
            return
        
        self.lidar_data = data.ranges 

    def dfs_step(self):
        # If we?re still moving, wait for that step to finish
        if not self.lidar_data:
            return
        if self.moving:
            if time.time() - self.move_start_time >= STEP_DURATION:
                self.stop_motion()
                self.current_node = self.move_target
                self.moving = False
            return

        # If stack is empty, we?re done
        if not self.dfs_stack:
            self.get_logger().info("DFS complete!")
            return

        current = self.dfs_stack[-1]

        # Mark the node on first visit
        if current not in self.marked:
            self.marked.add(current)

        # Try to push an unvisited neighbor
        for nbr in self.find_neighbors(current):
            if nbr not in self.marked and nbr not in self.dfs_stack:
                self.dfs_stack.append(nbr)
                self.move_to_node(current, nbr)
                return

        # No unvisited neighbors ? backtrack
        self.dfs_stack.pop()
        if self.dfs_stack:
            prev = self.dfs_stack[-1]
            self.move_to_node(current, prev)

    def find_neighbors(self, current):
        open_directions = self.check_for_walls()
        neighbors = []
        x, y = current

        for direction in open_directions:
            tempX = x
            tempY = y
            if direction == "N" and (self.heading==0 or self.heading==1 or self.heading==7) or direction == "NE" and (self.heading == 0 or self.heading==6 or self.heading==7) or direction == "E" and (self.heading==6 or self.heading==7 or self.heading==5) or direction=="SE" and (self.heading==4 or self.heading ==5 or self.heading==6) or direction=="S" and (self.heading==3 or self.heading==4 or self.heading==5) or direction=="SW" and (self.heading==3 or self.heading==4 or self.heading==2) or direction=="W" and (self.heading==3 or self.heading==1 or self.heading==2) or direction=="NW" and (self.heading==0 or self.heading==1 or self.heading==2):
                tempY += 1
            if direction == "E" and (self.heading==0 or self.heading==1 or self.heading==7) or direction == "SE" and (self.heading == 0 or self.heading==6 or self.heading==7) or direction == "S" and (self.heading==6 or self.heading==7 or self.heading==5) or direction=="SW" and (self.heading==4 or self.heading ==5 or self.heading==6) or direction=="W" and (self.heading==3 or self.heading==4 or self.heading==5) or direction=="NW" and (self.heading==3 or self.heading==4 or self.heading==2) or direction=="N" and (self.heading==3 or self.heading==1 or self.heading==2) or direction=="NE" and (self.heading==0 or self.heading==1 or self.heading==2):
                tempX += 1
            if direction == "S" and (self.heading==0 or self.heading==1 or self.heading==7) or direction == "SW" and (self.heading == 0 or self.heading==6 or self.heading==7) or direction == "W" and (self.heading==6 or self.heading==7 or self.heading==5) or direction=="NW" and (self.heading==4 or self.heading ==5 or self.heading==6) or direction=="N" and (self.heading==3 or self.heading==4 or self.heading==5) or direction=="NE" and (self.heading==3 or self.heading==4 or self.heading==2) or direction=="E" and (self.heading==3 or self.heading==1 or self.heading==2) or direction=="SE" and (self.heading==0 or self.heading==1 or self.heading==2):
                tempY -= 1
            if direction == "W" and (self.heading==0 or self.heading==1 or self.heading==7) or direction == "NW" and (self.heading == 0 or self.heading==6 or self.heading==7) or direction == "N" and (self.heading==6 or self.heading==7 or self.heading==5) or direction=="NE" and (self.heading==4 or self.heading ==5 or self.heading==6) or direction=="E" and (self.heading==3 or self.heading==4 or self.heading==5) or direction=="SE" and (self.heading==3 or self.heading==4 or self.heading==2) or direction=="S" and (self.heading==3 or self.heading==1 or self.heading==2) or direction=="SW" and (self.heading==0 or self.heading==1 or self.heading==2):
                tempX -= 1
            neighbors.append((tempX, tempY))

        return neighbors

    def move_to_node(self, previous, neighbor):
        dx = neighbor[0] - previous[0]
        dy = neighbor[1] - previous[1]

        # Determine turn direction
        if dx == 0 and dy == 1: #forward
            desired_heading = 0
        elif dx == 1 and dy == 1: 
            desired_heading = 1
        elif dx == 1 and dy == 0:
            desired_heading = 2
        elif dx == 1 and dy == -1:
            desired_heading = 3
        elif dx == 0 and dy == -1:
            desired_heading = 4
        elif dx == -1 and dy == -1:
            desired_heading = 5
        elif dx == -1 and dy == 0:
            desired_heading = 6
        elif dx == -1 and dy == 1:
            desired_heading = 7
        else:
            return

        turns = (desired_heading - self.heading) % 8
        if turns > 4:
            turns -= 8

        self.turn(turns)
        self.heading = desired_heading

        # Start forward motion
        self.twist.linear.x = MOVING_SPEED
        self.cmd_vel_pub.publish(self.twist)
        self.moving = True
        self.move_start_time = time.time()
        self.move_target = neighbor

    def turn(self, turns):
        if turns == 0:
            return
        twist = Twist()
        twist.angular.z = TURN_SPEED if turns > 0 else -TURN_SPEED
        duration = abs(turns) * TURN_DURATION
        self.cmd_vel_pub.publish(twist)
        time.sleep(duration)
        self.cmd_vel_pub.publish(Twist())

    def stop_motion(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def check_for_walls(self):
        if not self.lidar_data or len(self.lidar_data) < 360:
            return []

        angle_increment = 2 * np.pi / len(self.lidar_data)
        direction_angles = {
            "N": 0, #forward
            "NE": -np.pi / 4,
            "SE": -3 *np.pi / 4,
            "SW": 3 *np.pi / 4,
            "NW": np.pi / 4,
            "S": np.pi, #backward
            "E": -np.pi / 2, #right
            "W": np.pi / 2 #left
        }

        open_directions = []
        range_deg = 15
        range_rad = np.deg2rad(range_deg)
        range_indices = int(range_rad / angle_increment)

        for direction, angle in direction_angles.items():
            center_index = int((angle % (2 * np.pi)) / angle_increment)
            indices = [(center_index + i) % len(self.lidar_data) for i in range(-range_indices, range_indices + 1)]
            distances = [self.lidar_data[i] for i in indices if not np.isnan(self.lidar_data[i])]
            if distances and np.mean(distances) > WALL_DISTANCE:
                open_directions.append(direction)

        return open_directions

    def camera_callback(self, data):
        """Detect BLUE room"""
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
                    self.cmd_vel.publish(self.twist)
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