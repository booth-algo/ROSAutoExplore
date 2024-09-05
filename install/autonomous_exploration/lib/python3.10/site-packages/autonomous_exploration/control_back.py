from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq, math, random, yaml
import scipy.interpolate as si
import sys, threading, time
import rclpy

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class navigationControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription_map = self.create_subscription(OccupancyGrid, 'cmap', self.map_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.file_path = 'costmap.txt'
        self.map_data = None

        # Set up a timer to update the costmap every minute
        self.timer = self.create_timer(5.0, self.update_costmap)

        print("Exploration Mode Activated")

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        # Print odom_callback data
        print(f"Odometry Data: x={self.x}, y={self.y}, yaw={self.yaw}")

        # Map the odometry data to the costmap grid
        if self.map_data:
            grid_x, grid_y = self.world_to_grid(self.x, self.y)
            print(f"Mapped to grid: x={grid_x}, y={grid_y}")
            self.update_robot_position_in_grid(grid_x, grid_y)

    def scan_callback(self, msg):
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data
        print(f"[INFO] Map received: width={self.width}, height={self.height}, resolution={self.resolution}")

    def update_costmap(self):
        if self.map_data is not None:
            self.write_costmap_to_file(self.map_data)
        else:
            self.get_logger().warn("No map data available to write.")

    def write_costmap_to_file(self, msg):
        # Get the width and height of the occupancy grid
        width = msg.info.width
        height = msg.info.height

        # Open the file in write mode
        with open(self.file_path, 'w') as file:
            # Write the header and info for context
            # file.write(f'Header: {msg.header}\n')
            # file.write(f'Info: {msg.info}\n')
            
            # Initialize a grid with the same dimensions as the occupancy grid
            grid = np.zeros((height, width), dtype=int)
            for i in range(height):
                for j in range(width):
                    index = i * width + j
                    if msg.data[index] == -1:
                        grid[i, j] = 2
                    elif msg.data[index] > 0:
                        grid[i, j] = 1
                    else:
                        grid[i, j] = 0
            
            # Expand the walls
            expanded_grid = grid.copy()
            for i in range(height):
                for j in range(width):
                    if grid[i, j] == 1:
                        # Check the 8 neighboring cells
                        for di in [-1, 0, 1]:
                            for dj in [-1, 0, 1]:
                                ni, nj = i + di, j + dj
                                if 0 <= ni < height and 0 <= nj < width and grid[ni, nj] == 0:
                                    expanded_grid[ni, nj] = 3

            # Mark the robot's position
            if self.robot_grid_position is not None:
                rx, ry = self.robot_grid_position
                if 0 <= rx < height and 0 <= ry < width:
                    expanded_grid[rx, ry] = 4
            
            # Write the grid to the file
            for row in expanded_grid:
                file.write(' '.join(map(str, row)) + '\n')
        
        self.get_logger().info(f'Costmap data written to {self.file_path}')

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        # grid_x = int((x - self.originX) / self.resolution)
        # grid_y = int((y - self.originY) / self.resolution)

        grid_x = int((x-self.originX) / self.resolution)
        grid_y = int((y-self.originY) / self.resolution)

        print("self.originX:", self.originX, "self.originY:" ,self.originY, "self.resolution:", self.resolution)
        return grid_x, grid_y

    def update_robot_position_in_grid(self, grid_x, grid_y):
        """Update the robot's position in the occupancy grid."""
        self.robot_grid_position = (grid_x, grid_y)

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

        self.get_logger().info(f"Command sent: linear={linear}, angular={angular}")

def main(args=None):
    rclpy.init(args=args)
    exploration_node = navigationControl()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    def update_robot_position_in_grid(self, grid_x, grid_y):
        """Update the robot's position in the occupancy grid."""
        self.robot_grid_position = (grid_x, grid_y)

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

        self.get_logger().info(f"Command sent: linear={linear}, angular={angular}")

def main(args=None):
    rclpy.init(args=args)
    exploration_node = navigationControl()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()