import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
"""
class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos)

    def odom_callback(self, msg):
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular

        # Calculate the norm of the linear velocity
        linear_velocity_norm = np.linalg.norm([linear_velocity.x, linear_velocity.y, linear_velocity.z])

        # Get the third component of the angular velocity
        angular_velocity_z = angular_velocity.z
        # Write the velocities to a text file
        with open('velocities.txt', 'a') as f:
            f.write(f"{linear_velocity_norm} {angular_velocity_z}\n")

    def pose_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        plt.scatter(position.x, position.y)
        plt.pause(0.05)
        # Create rotation matrix from quaternion
        r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        rotation_matrix = r.as_matrix()

        # Create translation vector
        translation_vector = [position.x, position.y, position.z]
        
        # Create homogeneous transformation matrix
        homogeneous_transformation = np.eye(4)
        homogeneous_transformation[:3, :3] = rotation_matrix
        homogeneous_transformation[:3, 3] = translation_vector
        flattened_transformation = homogeneous_transformation[:3, :].flatten()

        # Append the transformation to the file
        with open('poses_home7.txt', 'ab') as f:
            np.savetxt(f, flattened_transformation[None, :], fmt='%.6f')

def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)

    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            qos)
        self.latest_odom = None
        self.latest_pose = None
        self.timer = self.create_timer(0.5, self.process_data)


    def odom_callback(self, msg):
        self.latest_odom = msg

    def pose_callback(self, msg):
        self.latest_pose = msg

    def process_data(self):
        if self.latest_odom is not None and self.latest_pose is not None:
            position = self.latest_pose.pose.pose.position
            orientation = self.latest_pose.pose.pose.orientation
            plt.scatter(position.x, position.y)
            plt.pause(0.05)
            # Create rotation matrix from quaternion
            r = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
            rotation_matrix = r.as_matrix()

            # Create translation vector
            translation_vector = [position.x, position.y, position.z]
            
            # Create homogeneous transformation matrix
            homogeneous_transformation = np.eye(4)
            homogeneous_transformation[:3, :3] = rotation_matrix
            homogeneous_transformation[:3, 3] = translation_vector
            flattened_transformation = homogeneous_transformation[:3, :].flatten()

            # Append the transformation to the file
            with open('poses_home7.txt', 'ab') as f:
                np.savetxt(f, flattened_transformation[None, :], fmt='%.6f')


            linear_velocity = self.latest_odom.twist.twist.linear
            angular_velocity = self.latest_odom.twist.twist.angular

            # Calculate the norm of the linear velocity
            linear_velocity_norm = np.linalg.norm([linear_velocity.x, linear_velocity.y, linear_velocity.z])

            # Get the third component of the angular velocity
            angular_velocity_z = angular_velocity.z
            # Write the velocities to a text file
            with open('velocities.txt', 'a') as f:
                f.write(f"{linear_velocity_norm} {angular_velocity_z}\n")

def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)

    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
