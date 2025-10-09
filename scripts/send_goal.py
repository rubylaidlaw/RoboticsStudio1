from enum import Enum, auto
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np

class WaypointManager:
    def __init__(self):
        # Predefined grid search waypoints
        self.grid_search_waypoints = [
            (10, 10),
            (-10, 8),
            (10, 6),
            (-10, 4),
            (10, 2),
            (-10, 0),
            (10, -2),
            (-10, -4),
            (10, -6),
            (-10, -8),
            (10, -10),
        ]
        self.index = 0

    def get_next_waypoint(self):
        if self.index >= len(self.grid_search_waypoints):
            self.index = 0 # Reset to start if all waypoints are done
        wp = self.grid_search_waypoints[self.index]
        self.index += 1
        return wp

    def reset(self):
        self.index = 0

# --- Robot Navigator ---
class RobotNavigator:
    def __init__(self):
        self.navigator = None
        self.current_yaw = 0.0
        self.waypoint_manager = WaypointManager()
        self.turn_angle = math.pi / 2

    def get_quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

    def run_navigation(self):
        rclpy.init()
        self.navigator = BasicNavigator()

        step_limit = 20  # max steps to avoid infinite loop for demo
        steps_taken = 0

        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.position.z = 0.0
        q = self.get_quaternion_from_yaw(0.0)  # face forward or whichever yaw
        init_pose.pose.orientation.x, init_pose.pose.orientation.y, init_pose.pose.orientation.z, init_pose.pose.orientation.w = q

        print("Navigating to home position (0,0)")
        self.navigator.goToPose(init_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.navigator)
        result = self.navigator.getResult()
        print(f"Arrived at home: (0,0), result: {result}")

        while steps_taken < step_limit:
            waypoint = self.waypoint_manager.get_next_waypoint()
            steps_taken += 1

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(waypoint[0])
            goal_pose.pose.position.y = float(waypoint[1])
            goal_pose.pose.position.z = 0.0

            q = self.get_quaternion_from_yaw(self.current_yaw)
            goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q

            print(f"Navigating to waypoint {waypoint}")
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator)

            result = self.navigator.getResult()
            print(f"Arrived at waypoint: {waypoint}, Result: {result}")

        rclpy.shutdown()

def main():
    robot = RobotNavigator()
    robot.run_navigation()


if __name__ == "__main__":
    main()