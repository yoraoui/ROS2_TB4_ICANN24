import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            qos)
        self.subscription

    def listener_callback(self, msg):
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        # Calculate the norm of the linear velocity
        linear_velocity_norm = np.linalg.norm([linear_velocity.x, linear_velocity.y, linear_velocity.z])

        # Get the third component of the angular velocity
        angular_velocity_z = angular_velocity.z

        # Write the velocities to a text file
        with open('velocities.txt', 'a') as f:
            f.write(f"{linear_velocity_norm} {angular_velocity_z}\n")

def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)

    velocity_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()