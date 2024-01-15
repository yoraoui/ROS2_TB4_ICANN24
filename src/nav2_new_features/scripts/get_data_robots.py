import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from datetime import datetime
import matplotlib.pyplot as plt
import os
class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
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
        self.latest_velocity = None
        self.latest_pose = None
        self.experience_dir = os.path.join('experiences', datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
        os.makedirs(self.experience_dir, exist_ok=True)
        # Create a timer callback that calls process_data every 0.5 seconds
        self.timer = self.create_timer(0.5, self.process_data)
        current_time = datetime.now()
        self.timestamp = current_time.timestamp()
        
    def odom_callback(self, msg):
        self.latest_velocity = msg
    def pose_callback(self, msg):
        self.latest_pose = msg.pose.pose
    def process_data(self):
        if self.latest_velocity is not None and self.latest_pose is not None:
            current_time = datetime.now()
            timestamp = current_time.timestamp()
            # Process the latest velocity and pose data
            self.get_logger().info('Latest velocity: "%s"' % self.latest_velocity)
            self.get_logger().info('Latest pose: "%s"' % self.latest_pose)
            linear_velocity = self.latest_velocity.twist.twist.linear
            angular_velocity =  self.latest_velocity.twist.twist.angular
            # Calculate the norm of the linear velocity
            linear_velocity_norm = np.linalg.norm([linear_velocity.x, linear_velocity.y, linear_velocity.z])
            # Get the third component of the angular velocity
            angular_velocity_z = angular_velocity.z
            velocities = np.array([linear_velocity_norm, angular_velocity_z])
            position = self.latest_pose.position
            orientation = self.latest_pose.orientation
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
            position_odom = self.latest_velocity.pose.pose.position
            orientation_odom =  self.latest_velocity.pose.pose.orientation
            r_odom = R.from_quat([orientation_odom.x, orientation_odom.y, orientation_odom.z, orientation_odom.w])
            rotation_matrix_odom = r_odom.as_matrix()
            # Create translation vector
            translation_vector_odom = [position_odom.x, position_odom.y, position_odom.z]
            # Create homogeneous transformation matrix
            homogeneous_transformation_odom = np.eye(4)
            homogeneous_transformation_odom[:3, :3] = rotation_matrix_odom
            homogeneous_transformation_odom[:3, 3] = translation_vector_odom
            flattened_transformation_odom = homogeneous_transformation_odom[:3, :].flatten()
            with open(os.path.join(self.experience_dir, 'orientation_z.txt'), 'ab') as f:
                np.savetxt(f, [orientation.z], fmt='%.6f')
            with open(os.path.join(self.experience_dir, 'times.txt'), 'ab') as f:
                np.savetxt(f, [timestamp], fmt='%.6f')
            with open(os.path.join(self.experience_dir, 'velocities.txt'), 'ab') as f:
                np.savetxt(f, velocities[None, :], fmt='%.6f')
            with open(os.path.join(self.experience_dir, 'poses.txt'), 'ab') as f:
                np.savetxt(f, flattened_transformation[None, :], fmt='%.6f')
            with open(os.path.join(self.experience_dir, 'poses_odom.txt'), 'ab') as f:
                np.savetxt(f, flattened_transformation_odom[None, :], fmt='%.6f')
def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()