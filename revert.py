#!/usr/bin/env python3

class DisjointSet:
    def __init__(self, size):
        self.vertex = [i for i in range(size)]
        self.weight = [1] * size
        self._size = size

    def validate(self, v1):
        #Check if v1 is a valid index
        return self._size > v1 and v1 >= 0

    def size(self, v1):
        #return the size of the set v1 belongs to
        if self.validate(v1): #check to see if it's a valid index
            return self.weight[self.find(v1)] #return size
        return 0    

    def parent(self, v1):
        #return the parent of v1... if v1 is the root of a tree, return the negative size of the tree for which v1 is the root
        if self.validate(v1): #check to see if it's a valid index
            if self.vertex[v1] == v1:
                return -self.weight[v1]
            return self.vertex[v1]

    def isConnected(self, v1, v2):
        #check if given 2 vertex are connected
        if self.validate(v1) and self.validate(v2):
            #find the root of both sets
            parent1 = self.find(v1)
            parent2 = self.find(v2)
            #if v1 and v2 have the same roots, they're connected
            return parent1 == parent2

    def find(self, v1):
        #returns the root of the set v1 belongs to
        if self.validate(v1):
            #hold the index of the parent
            prev = v1
            parent = self.parent(v1)
            while parent >= 0:
                prev = parent
                parent = self.parent(parent)
            return prev
       
    def unionByWeight(self, v1, v2):
        #connects two elements v1 and v2 together based on weight
        root1 = self.find(v1)
        root2 = self.find(v2)
        if self.validate(v1) and self.validate(v2) and v1 != v2 and root1 != root2:
            #if weight v1 > v2 then v2 merges into v1; root of v2 merges with root of v1
            if self.size(v1) > self.size(v2):
                self.weight[root1] += self.size(v2) #increase the weight of the root
                self.vertex[root2] = root1 #change the parent of root2 to be root1
            else:
                self.weight[root2] += self.size(v1) #increase the weight of the root
                self.vertex[root1] = root2


import sys

class Graph:
    def __init__(self):
        # len(self.graph) = vertices
        # self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]
        self.graph = {} # key = node, val = dictionary of neighbor and weight
        self.marked = [] #make a list to keep track of vertices we've marked for traversals

    def add_edge(self, u, v, w):
        # u and v are vertices, w is weight
        if u not in self.graph:
            self.graph[u] = {}
        if v not in self.graph:
            self.graph[v] = {}
        self.graph[u][v] = w
        self.graph[v][u] = w

    def dfs(self, start): 
        self.marked = [] #make the marked list empty
        return self._dfs(start)

    def _dfs(self, current): #recursive private function for depth first search
        #first, mark the current node
        self.marked.append(current)
        #now, search through every neighbor
        for neighbor in self.graph[current]:
            #check to see if we've already visited that neighbor
            if neighbor not in self.marked:
                self._dfs(neighbor)
        return self.marked
    
    def bfs(self, start): 
        self.marked = [] #nothing is marked
        queue = [start] #have the start initially on the queue
        current = start #initialize a current vertex
        while len(queue) != 0: #while the queue is not empty
            current = queue.pop(0) #pop the front
            self.marked.append(current) #mark the current one
            for neighbor in self.graph[current]: 
            	if neighbor not in self.marked and neighbor not in queue:
                    queue.append(neighbor) #add every neighbor we haven't already done to the queue
        return self.marked

    def dijkstra(self, src):
        # Stores shortest distance.
        dist = {node: sys.mazsize for node in self.graph}
        # Shortest distance to the same node is 0.
        dist[src] = 0

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
MOVING_SPEED = 0.3
WALL_DISTANCE = 0.3  # Minimum distance from wall (meters) (4 in ~ 0.1 m)
TURN_SPEED = 1.0  # Speed for turning away from walls
FORWARD_DISTANCE = 0.3  # Distance to move forward (meters)

END_DISTANCE = 0.1 # Minimum distance from blue wall (meters) (4 in ~ 0.1 m)
FORWARD_SPEED = 0.2
COLOR_TOLERANCE = 20  # For color detection centering

TURN_DURATION = 1.57 # figure out what 90 egree is
STEP_DURATION = 0.15

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
        
        self.marked = set()
        self.dfs_stack = [self.current_node]
        self.backtrack_stack = []


        self.moving = False
        self.move_start_time = None
        self.move_target = None

        self.create_timer(0.15, self.dfs_step)
        
        

    def lidar_callback(self, data):
        """Process Lidar data for wall detection and navigation"""
        if self.at_end or not self.first_attempt:
            return
        
        self.lidar_data = data.ranges 
        
        


    def dfs_step(self):
        if self.at_end or not self.lidar_data:
            return

        if self.moving:
            if time.time() - self.move_start_time >= STEP_DURATION:
                self.stop_motion()
                self.current_node = self.move_target
                self.moving = False
            return

        if not self.dfs_stack:
            self.get_logger().info("DFS complete!")
            return

        current = self.dfs_stack[-1]
        if current not in self.marked:
            self.marked.add(current)
            neighbors = self.find_neighbors(current)
            for neighbor in neighbors:
                self.graph.add_edge(current, neighbor, -1)
                if neighbor not in self.marked and neighbor not in self.dfs_stack:
                    self.dfs_stack.append(neighbor)
                    self.move_to_node(current, neighbor)
                    return
        else:
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
            difference = self.heading - direction
            if difference == 0:
                tempY += 1
            elif difference == -1 or difference == 3:
                tempX -= 1
            elif abs(difference) == 2:
                tempY -= 1
            elif difference == -3 or difference == 1:
                tempX += 1
            neighbors.append((tempX, tempY))
                
        print(neighbors)

        return neighbors
        
    def move_to_node(self, previous, neighbor):
        dx = neighbor[0] - previous[0]
        dy = neighbor[1] - previous[1]

        # Determine turn direction
        if dx == 1:
            desired_heading = 1
        elif dx == -1:
            desired_heading = 3
        elif dy == 1:
            desired_heading = 0
        elif dy == -1:
            desired_heading = 2
        else:
            return

        turns = (desired_heading - self.heading) % 4
        if turns == 3:
            turns = -1

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
        twist.angular.z = -TURN_SPEED if turns > 0 else TURN_SPEED
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
            0: 0,
            3: -np.pi / 2,
            1: np.pi / 2
        }

        open_directions = []
        range_deg = 15
        range_rad = np.deg2rad(range_deg)
        range_indices = int(range_rad / angle_increment)

        for direction, angle in direction_angles.items():
            center_index = int((angle % (2 * np.pi)) / angle_increment)
            indices = [(center_index + i) % len(self.lidar_data) for i in range(-range_indices, range_indices + 1)]
            distances = [self.lidar_data[i] for i in indices if not np.isnan(self.lidar_data[i])]
            if distances and np.min(distances) > WALL_DISTANCE:
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
