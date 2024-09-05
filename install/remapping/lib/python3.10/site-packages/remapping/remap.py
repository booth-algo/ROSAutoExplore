import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapRemapper(Node):

    def __init__(self):
        super().__init__('map_remapper')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(OccupancyGrid, '/cmap', 10)

    def listener_callback(self, map_msg):
        M = 75  # Threshold for obstacles
        N = 50  # Threshold for unknown areas
        data = list(map_msg.data)
        for y in range(map_msg.info.height):
            for x in range(map_msg.info.width):
                i = x + (map_msg.info.height - 1 - y) * map_msg.info.width
                if data[i] >= M:
                    data[i] = 100  # Mark as obstacle
                elif 0 <= data[i] < N:
                    data[i] = 0  # Mark as free
                else:
                    data[i] = -1  # Mark as unknown
        map_msg.data = tuple(data)
        self.publisher.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
