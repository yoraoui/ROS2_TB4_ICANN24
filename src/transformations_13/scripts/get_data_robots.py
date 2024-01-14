from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
import tf2_ros
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self):
        try:
            # Get the transformation from the map frame to the base_link frame
            # Wait for up to 5.0 seconds for the transformation to become available
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

            # The transform object contains the translation and rotation from the map frame to the base_link frame
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            time = transform.header.stamp

            self.get_logger().info('Translation: %s', translation)
            self.get_logger().info('Rotation: %s', rotation)
            self.get_logger().info('Time: %s', time)

            return translation, rotation, time
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print("hi")
            return None, None, None        
# ...

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    navigator.get_transform()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()