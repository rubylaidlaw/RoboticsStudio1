from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import numpy as np

def get_quaternion_from_yaw(yaw):
    return [0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)]

rclpy.init()
navigator = BasicNavigator()

# Wait for NAV2 to activate (navigation stack ready, no AMCL dependency)
navigator.waitUntilNav2Active()

x0, y0 = 0.0, 0.0
distance = 2.0
yaw = 0.0

goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = x0
goal_pose.pose.position.y = y0 + distance
goal_pose.pose.position.z = 0.0

q = get_quaternion_from_yaw(yaw)
goal_pose.pose.orientation.x = q[0]
goal_pose.pose.orientation.y = q[1]
goal_pose.pose.orientation.z = q[2]
goal_pose.pose.orientation.w = q[3]

navigator.goToPose(goal_pose)

while not navigator.isTaskComplete():
    rclpy.spin_once(navigator)

result = navigator.getResult()
print("Navigation complete with result:", result)
rclpy.shutdown()
