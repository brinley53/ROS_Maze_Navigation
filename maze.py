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
TURN_SPEED = 0.5  # Speed for turning away from walls
FORWARD_DISTANCE = 0.3  # Distance to move forward (meters)

END_DISTANCE = 0.1 # Minimum distance from blue wall (meters) (4 in ~ 0.1 m)
FORWARD_SPEED = 0.2
COLOR_TOLERANCE = 20  # For color detection centering

TURN_DURATION = 1.57 # maybe 90 degrees?
STEP_DURATION = 0.635 # maybe 5 inches?

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
        
        # State management
        self.following_wall = False
        self.first_attempt = True
        self.mapping_complete = False

        self.twist = Twist()


    def lidar_callback(self, data):
        """Process Lidar data for wall detection and navigation"""
        if self.at_end or not self.first_attempt:
            return
        
        self.lidar_data = data.ranges 
        if self.start:
            self.dfs((0, 0))
            self.start = False

    def bfs(self):
        # check directions
        self.marked = [] #nothing is marked
        queue = [self.current_node]

        while len(queue) != 0:
            self.current_node = queue.pop(0) #pop the front
            self.marked.append(self.current_node)
            self.move_to_node()
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)
            neighbors = self.find_neighbors()
            for neighbor in neighbors:
                self.graph.add_edge(self.current_node, neighbor, -1)
                if neighbor not in self.marked and neighbor not in queue:
                    queue.append(neighbor)
            self.previous_node = self.current_node

    def dfs(self, start): 
        self.marked = [] #make the marked list empty
        return self._dfs(start)

    def _dfs(self, current): #recursive private function for depth first search
        #first, mark the current node
        self.marked.append(current)
        #now, search through every neighbor
        neighbors = self.find_neighbors()
        for neighbor in neighbors:
            self.graph.add_edge(self.current_node, neighbor, -1)
            #check to see if we've already visited that neighbor
            if neighbor not in self.marked:
                self.move_to_node(current, neighbor)
                self._dfs(neighbor)
                self.return_to_node(neighbor, current)
        return self.marked

    def move_to_node(self, current, neighbor):
        if (current == neighbor):
            return
        current_x, current_y = current
        previous_x, previous_y = neighbor

        if current_x - previous_x > 0:
            # move right
            self.twist.angular.z = -TURN_SPEED
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(TURN_DURATION)
        elif current_x - previous_x < 0:
            # move left
            self.twist.angular.z = TURN_SPEED
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(TURN_DURATION)
        elif current_y - previous_y < 0:
            # move backward
            self.twist.angular.z = -TURN_SPEED
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(TURN_DURATION*2)

        self.twist.angular.z = 0.0
        self.twist.linear.x = MOVING_SPEED
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(STEP_DURATION)

    def find_neighbors(self):
        # this works correctly
        open_directions = self.check_for_walls()
        neighbors = []
        current_x, current_y = self.current_node

        for direction in open_directions:
            neighbor = None
            match direction:
                case 'forward':
                    neighbor = (current_x, current_y + 1)
                case 'backward':
                    neighbor = (current_x, current_y - 1)
                case 'left':
                    neighbor = (current_x - 1, current_y)
                case 'right':
                    neighbor = (current_x + 1, current_y)
            if neighbor != None:
                neighbors.append(neighbor)

        return neighbors
        
    def check_for_walls(self):
        #this gets the correct front/back/side but make sure that the ranges work
        #B attempt
        if not self.lidar_data or len(self.lidar_data) < 360:
            return []

        angle_increment = 2 * np.pi / len(self.lidar_data)  # Full circle / number of measurements
        direction_angles = {
            "forward": 0,
            "right": -np.pi / 2,
            # "backward": np.pi, don't need backward
            "left": np.pi / 2
        }

        open_directions = []
        range_deg = 10  # +/- 10 degrees around each direction
        range_rad = np.deg2rad(range_deg)
        range_indices = int(range_rad / angle_increment)

        for direction, angle in direction_angles.items():
            # Get central index for this direction
            center_index = int((angle % (2 * np.pi)) / angle_increment)
            
            # Compute indices to average
            indices = [(center_index + i) % len(self.lidar_data) for i in range(-range_indices, range_indices + 1)]
            
            # Average the distances in this range
            distances = [self.lidar_data[i] for i in indices if not np.isnan(self.lidar_data[i])]
            
            if distances and np.mean(distances) > WALL_DISTANCE:
                open_directions.append(direction)
        
        return open_directions
            
    def find_wall(self):
        """Rotate in place until a wall is detected to begin wall-following"""
        min_distance = min(self.lidar_data)
        if min_distance < WALL_DISTANCE:
            self.following_wall = True
            self.get_logger().info("Wall found, starting wall following")
            return

        self.twist.angular.z = 0.3  # Rotate to find wall
        self.cmd_vel_pub.publish(self.twist)
        
    def explore_with_wall_following(self):
        """Use right hand wall method"""
        front_dist = min(min(self.lidar_data[:30]), min(self.lidar_data[330:]))
        right_dist = min(self.lidar_data[240:300])
        left_dist = min(self.lidar_data[60:120])
        
        # Wall following logic
        if front_dist < WALL_DISTANCE:
            self.twist.angular.z = TURN_SPEED  # Turn left
            self.twist.linear.x = 0.0
            self.record_turn('left')
        elif right_dist > WALL_DISTANCE * 1.5:
            self.twist.angular.z = -TURN_SPEED * 0.5  # Turn right slightly
            self.twist.linear.x = FORWARD_SPEED * 0.5
        elif right_dist < WALL_DISTANCE * 0.7:
            self.twist.angular.z = TURN_SPEED * 0.3  # Turn left slightly
            self.twist.linear.x = FORWARD_SPEED * 0.7
        else:
            self.twist.linear.x = FORWARD_SPEED  # Move forward
            self.twist.angular.z = 0.0
            self.record_straight()

        self.cmd_vel.publish(self.twist)

    def record_position(self):
        """Record current node and connections in graph"""
        front_clear = min(min(self.lidar_data[:30]), min(self.lidar_data[330:])) > WALL_DISTANCE * 1.5
        right_clear = min(self.lidar_data[240:300]) > WALL_DISTANCE * 1.5
        left_clear = min(self.lidar_data[60:120]) > WALL_DISTANCE * 1.5
        
        directions = {
            'front': front_clear,
            'right': right_clear,
            'left': left_clear,
        }
        
        for dir_name, is_clear in directions.items():
            if is_clear:
                neighbor = self.get_neighbor_coordinates(self.current_node, self.heading, dir_name)
                weight = -1 #update this to actually include edge weight
                self.graph.add_edge(self.current_node, neighbor, weight)

    def get_neighbor_coordinates(self, node, heading, direction):
        x, y = node
        dir_map = {
            0: {'front': (x, y + 1), 'left': (x - 1, y), 'right': (x + 1, y)},
            1: {'front': (x + 1, y), 'left': (x, y + 1), 'right': (x, y - 1)},
            2: {'front': (x, y - 1), 'left': (x + 1, y), 'right': (x - 1, y)},
            3: {'front': (x - 1, y), 'left': (x, y - 1), 'right': (x, y + 1)},
        }
        return dir_map[heading][direction]

    def record_turn(self, direction):
        """Update position after a turn"""
        if direction == 'left':
            self.heading = (self.heading - 1) % 4
        elif direction == 'right':
            self.heading = (self.heading + 1) % 4

    def record_straight(self):
        """Update position after moving straight"""
        x, y = self.current_node
        # Put nswe directions for now to get a rough idea of the map
        # North is the front of wherever Mox is facing after placing
        if self.heading == 0:  #north
            self.current_node = (x, y + 1)
        elif self.heading == 1:  #east
            self.current_node = (x + 1, y)
        elif self.heading == 2:  #south
            self.current_node = (x, y - 1)
        elif self.heading == 3:  #west
            self.current_node = (x - 1, y)

    # for second attempt, separate file
    # def plan_path(self):
    #     """2nd ATTEMPT: after mapping using BFS"""
    #     # need to change this to be a separate file
    #     start = (0, 0)
    #     end = self.find_farthest_point()
        
    #     if end is None:
    #         self.get_logger().warn("Could not determine end point, using farthest point")
    #         end = self.find_farthest_point()
        
    #     self.path = self.graph.bfs(start, end)
        
    #     if self.path:
    #         self.get_logger().info(f"Path planned: {self.path}")
    #         self.path_index = 0
    #     else:
    #         self.get_logger().error("No path found to BLUE room")

    # for second attempt, separate file
    # def follow_path(self, twist):
    #     """Follow the planned BFS path"""
    #     if not self.path or self.path_index >= len(self.path):
    #         return
            
    #     target_node = self.path[self.path_index]
        
    #     if self.current_node == target_node:
    #         self.path_index += 1
    #         if self.path_index >= len(self.path):
    #             return
    #         target_node = self.path[self.path_index]
            
    #     self.move_to_node(twist, target_node)

    # for second attempt, different file
    # def move_to_node(self, twist, target_node):
    #     """Move toward the target node"""
    #     current_x, current_y = self.current_node
    #     target_x, target_y = target_node

    #     '''
    #     It should go something like this
    #     Turn Decisions:
    #     Current | Target | Turn
    #     -----------------------------
    #     North   | East   | Right 
    #     East    | South  | Right   
    #     South   | West   | Right 
    #     West    | North  | Right 
    #     North   | West   | Left 

    #         0
    #     3 <-  -> 1   NSWE directions look like this
    #         2
    #     '''
        
    #     # Determine required movement
    #     # Need to move North
    #     if target_y > current_y:
    #         if self.heading == 0:  # Already facing North
    #             twist.linear.x = FORWARD_SPEED
    #         elif self.heading == 1:  # Facing East
    #             twist.angular.z = TURN_SPEED  # Turn left
    #         elif self.heading == 2:  # Facing South
    #             twist.angular.z = TURN_SPEED  # Turn left (or right, could do 180)
    #         elif self.heading == 3:  # Facing West
    #             twist.angular.z = -TURN_SPEED  # Turn right
    #      # Need to move East
    #     elif target_x > current_x:
    #         if self.heading == 0:  # Facing North
    #             twist.angular.z = -TURN_SPEED  # Turn right
    #         elif self.heading == 1:  # Already facing East
    #             twist.linear.x = FORWARD_SPEED
    #         elif self.heading == 2:  # Facing South
    #             twist.angular.z = TURN_SPEED  # Turn left
    #         elif self.heading == 3:  # Facing West
    #             twist.angular.z = TURN_SPEED  # Turn left (or right, could do 180)
    #     # Need to move South
    #     elif target_y < current_y:
    #         if self.heading == 0:  # Facing North
    #             twist.linear.x = 0.0
    #             twist.angular.z = TURN_SPEED  # Turn left (180° turn)
    #         elif self.heading == 1:  # Facing East
    #             twist.linear.x = 0.0
    #             twist.angular.z = -TURN_SPEED  # Turn right
    #         elif self.heading == 2:  # Already facing South
    #             twist.linear.x = FORWARD_SPEED
    #             twist.angular.z = 0.0
    #         elif self.heading == 3:  # Facing West
    #             twist.linear.x = 0.0
    #             twist.angular.z = TURN_SPEED  # Turn left 
    #     # Need to move West
    #     elif target_x < current_x:
    #         if self.heading == 0:  # Facing North
    #             twist.linear.x = 0.0
    #             twist.angular.z = TURN_SPEED  # Turn left
    #         elif self.heading == 1:  # Facing East
    #             twist.linear.x = 0.0
    #             twist.angular.z = TURN_SPEED  # Turn left (180° turn)
    #         elif self.heading == 2:  # Facing South
    #             twist.linear.x = 0.0
    #             twist.angular.z = -TURN_SPEED  # Turn right
    #         elif self.heading == 3:  # Already facing West
    #             twist.linear.x = FORWARD_SPEED
    #             twist.angular.z = 0.0
                
    #     else:  # Already at target position
    #         twist.linear.x = 0.0
    #         twist.angular.z = 0.0
    #         self.path_index += 1  # Move to next node in path

    # for second attempt, dif file
    # def find_farthest_point(self):
    #     """Find the farthest explored point from start"""
    #     start = (0, 0)
    #     visited = set()
    #     queue = deque([(start, 0)])
    #     farthest = (start, 0)

    #     while queue:
    #         node, dist = queue.popleft()
    #         if dist > farthest[1]:
    #             farthest = (node, dist)
    #         for neighbor in self.graph.adj.get(node, []):
    #             if neighbor not in visited:
    #                 visited.add(neighbor)
    #                 queue.append((neighbor, dist + 1))
    #     return farthest[0]

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
