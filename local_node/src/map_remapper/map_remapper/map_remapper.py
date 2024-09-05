#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRemapper(Node):
    def __init__(self):
        super().__init__('map_remapper')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/cmap', 10)

    def map_callback(self, msg):
        binary_map = OccupancyGrid()
        binary_map.header = msg.header
        binary_map.info = msg.info
        binary_map.data = [0] * len(msg.data)  # Initialize with zeros

        # Threshold to convert probability values to binary values
        threshold = 50  # Assuming probabilities are in [0, 100]

        for i in range(len(msg.data)):
            if msg.data[i] == -1:  # Unknown value in ROS occupancy grid
                binary_map.data[i] = -1
            elif msg.data[i] > threshold:
                binary_map.data[i] = 100  # Occupied
            else:
                binary_map.data[i] = 0  # Free

        self.publisher_.publish(binary_map)

def main(args=None):
    rclpy.init(args=args)
    map_remapper = MapRemapper()
    rclpy.spin(map_remapper)
    map_remapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
