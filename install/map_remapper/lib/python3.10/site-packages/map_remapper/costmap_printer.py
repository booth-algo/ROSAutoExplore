#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class CostmapPrinter(Node):
    def __init__(self):
        super().__init__('costmap_printer')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/cmap',
            self.costmap_callback,
            10)
        self.file_path = 'costmap.txt'

    def costmap_callback(self, msg):
        # Get the width and height of the occupancy grid
        width = msg.info.width
        height = msg.info.height

        # Open the file in write mode
        with open(self.file_path, 'w') as file:
            # Write the header and info for context
            file.write(f'Header: {msg.header}\n')
            file.write(f'Info: {msg.info}\n')
            
            # Initialize a grid with the same dimensions as the occupancy grid
            grid = []
            for i in range(height):
                row = []
                for j in range(width):
                    index = i * width + j
                    if msg.data[index] == -1:
                        row.append('2')
                    elif msg.data[index] > 0:
                        row.append('1')
                    else:
                        row.append('0')
                grid.append(row)
            
            # Write the grid to the file
            for row in grid:
                file.write(' '.join(row) + '\n')
        
        self.get_logger().info(f'Costmap data written to {self.file_path}')

def main(args=None):
    rclpy.init(args=args)
    costmap_printer = CostmapPrinter()
    rclpy.spin(costmap_printer)
    costmap_printer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
