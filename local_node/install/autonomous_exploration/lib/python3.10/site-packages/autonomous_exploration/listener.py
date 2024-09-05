import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import numpy as np

def quaternion_to_matrix(quat):
    """Convert a quaternion into a rotation matrix."""
    w, x, y, z = quat
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w_new = -(w1*w2 - x1*x2 - y1*y2 - z1*z2)
    z_new = (w1*x2 + x1*w2 + y1*z2 - z1*y2)
    x_new = 0
    y_new = 0
    #w1*y2 - x1*z2 + y1*w2 + z1*x2
    #w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([x_new, y_new, z_new, w_new])

class Tf2Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.publisher = self.create_publisher(TransformStamped, 'listenbaby', 10)

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            trans_map_base = self.tf_buffer.lookup_transform('map', 'base_link', now)
            
            self.get_logger().info('Map to Base Link Translation: %s' % str(trans_map_base.transform.translation))
            self.get_logger().info('Map to Base Link Rotation: %s' % str(trans_map_base.transform.rotation))

            # Example odom data (replace with actual data)
            odom_position = np.array([0.0, 0.0, 0.0])  # Replace with actual odom position data
            odom_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Replace with actual odom orientation quaternion

            # Translation from map to base_link
            translation_map_base = np.array([
                trans_map_base.transform.translation.x,
                trans_map_base.transform.translation.y,
                trans_map_base.transform.translation.z
            ])

            # Rotation (quaternion) from map to base_link
            rotation_map_base = np.array([
                trans_map_base.transform.rotation.x,
                trans_map_base.transform.rotation.y,
                trans_map_base.transform.rotation.z,
                trans_map_base.transform.rotation.w
            ])

            # Transform position from map to base_link
            rotation_matrix_map_base = quaternion_to_matrix(rotation_map_base)
            base_link_position = np.dot(rotation_matrix_map_base, odom_position) + translation_map_base

            # Transform orientation from map to base_link
            base_link_orientation = quaternion_multiply(rotation_map_base, odom_orientation)

            self.get_logger().info('Base_link position: %s' % str(base_link_position))
            self.get_logger().info('Base_link orientation: %s' % str(base_link_orientation))

            # Create and publish TransformStamped message
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = now.to_msg()
            transform_stamped.header.frame_id = 'map'
            transform_stamped.child_frame_id = 'base_link'
            transform_stamped.transform.translation.x = base_link_position[0]
            transform_stamped.transform.translation.y = base_link_position[1]
            transform_stamped.transform.translation.z = base_link_position[2]
            transform_stamped.transform.rotation.x = base_link_orientation[0]
            transform_stamped.transform.rotation.y = base_link_orientation[1]
            transform_stamped.transform.rotation.z = base_link_orientation[2]
            transform_stamped.transform.rotation.w = base_link_orientation[3]

            self.publisher.publish(transform_stamped)
            self.get_logger().info('Published transform to listenbaby topic')

        except Exception as e:
            self.get_logger().info('Could not transform: %s' % str(e))

def main():
    rclpy.init()
    node = Tf2Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
