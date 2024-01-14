import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
import tf2_ros
#import Buffer
from tf2_ros import Buffer, TransformListener
# Shelf positions for picking
shelf_positions = {
        "shelf_A": [2.95, 0.0  ],
        "shelf_B": [3.006423,  1.83 ],
        "shelf_C": [1.44830 , 2.118003 ],
        "shelf_D": [1.443652 ,3.85923 ]}
# Shipping destination for picked products
shipping_destinations = {
    "door": [3.000264, -0.000431 ],
    "toilet": [3.146376 , 2.170780 ],
    "miror": [1.162124 ,2.307672 ],
    "television": [1.298566 ,  3.327468]}

def main():
    request_item_location = 'shelf_A'
    request_destination = 'door'
    rclpy.init()
    navigator = BasicNavigator()
    initial_pose = PoseStamped() 
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)
    # Wait for navigation to activate fully
    #navigator.waitUntilNav2Active()
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = math.sin(math.pi / 4)
    shelf_item_pose.pose.orientation.w = math.cos(math.pi / 4)
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    # go to toilet

    request_item_location = 'shelf_B'
    request_destination = 'toilet'

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = math.sin(math.pi / 4)
    shelf_item_pose.pose.orientation.w = math.cos(math.pi / 4)
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)
    request_item_location = 'shelf_C'
    request_destination = 'miror'

    # Wait for navigation to activate fully
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = math.sin(math.pi / 4)
    shelf_item_pose.pose.orientation.w = math.cos(math.pi / 4)
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)

    request_item_location = 'shelf_D'

    request_destination = 'television'

    # Wait for navigation to activate fully
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = math.sin(math.pi / 4)
    shelf_item_pose.pose.orientation.w = math.cos(math.pi / 4)
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)


    # Do something during your route
    # (e.x. queue up future tasks or detect person for fine-tuned positioning)
    # Print information for workers on the robot's ETA for the demonstration
    """
    i=0
    while  not navigator.isTaskComplete():
        i = i+1
        feedback = navigator.getFeedback()
        if feedback and i%5==0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')        
    result = navigator.getResult()
    """
if __name__ == '__main__':
    main()
