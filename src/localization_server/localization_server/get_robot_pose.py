import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Robot pose: "%s"' % msg.pose.pose)

def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    rclpy.spin(pose_subscriber)

    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()