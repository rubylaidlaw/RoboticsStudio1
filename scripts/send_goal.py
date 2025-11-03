from enum import Enum, auto
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, Float32
import numpy as np
import random

class Estop:
    def __init__(self):
        self.estop = False
        self.saved_goal = None

    def set_estop(self, status: bool):
        if status != self.estop:
            self.estop = status
            if status:
                print("Emergency Stop enabled! Robot motions will pause.")
            else:
                print("Emergency Stop released! Resuming robot motions.")

    def wait_if_estop(self):
        while self.estop:
            time.sleep(0.1)

    def is_active(self):
        return self.estop

class WaypointManager:
    def __init__(self):
        self.x_min, self.x_max = -10, 10
        self.y_min, self.y_max = -10, 10
        self.generated_waypoints = []

    def get_next_waypoint(self):
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        waypoint = (x, y)
        self.generated_waypoints.append(waypoint)
        return waypoint

    def print_generated_waypoints(self):
        print("Generated Waypoints:")
        for i, wp in enumerate(self.generated_waypoints):
            print(f"Waypoint {i+1}: ({wp[0]:.2f}, {wp[1]:.2f})")

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.navigator = BasicNavigator()
        self.current_yaw = 0.0
        self.waypoint_manager = WaypointManager()
        self.estop = Estop()
        self.current_goal = None
        self.estop_subscription = self.create_subscription(
            Bool,
            '/estop_status',
            self.estop_callback,
            10
        )
        self.get_logger().info("Robot Navigator initialized, listening for e-stop...")
        self.distance_pub = self.create_publisher(Float32, '/distance_to_goal', 10)

    def estop_callback(self, msg: Bool):
        self.estop.set_estop(msg.data)

    def get_quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = self.get_quaternion_from_yaw(yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose

    def navigate_with_estop_support(self, goal_pose, description=""):
        self.current_goal = goal_pose
        print(f"Navigating to {description}")
        self.navigator.goToPose(goal_pose)
        last_print_time = 0

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, "distance_remaining"):
                dist = feedback.distance_remaining
            else:
                dist = self.distance_to_goal(self.current_goal)

            self.distance_pub.publish(Float32(data=dist))

            current_time = time.time()
            if current_time - last_print_time > 1.0:
                print(f"Distance to {description}: {dist:.2f} meters")
                last_print_time = current_time

            if self.estop.is_active():
                print("E-Stop activated! Canceling navigation...")
                self.navigator.cancelTask()
                self.estop.wait_if_estop()
                print(f"E-Stop released. Resuming navigation to {description}")
                self.navigator.goToPose(self.current_goal)

            # Generate and move to a new waypoint if close enough
            if dist < 2.0:
                waypoint = self.waypoint_manager.get_next_waypoint()
                new_goal_pose = self.create_pose(waypoint[0], waypoint[1], self.current_yaw)
                self.navigator.goToPose(new_goal_pose)
                self.current_goal = new_goal_pose  # Update current_goal for feedback
                description = f"waypoint {waypoint}"
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Arrived at {description}")
        elif result == TaskResult.CANCELED:
            print(f"Navigation to {description} was canceled")
        elif result == TaskResult.FAILED:
            print(f"Navigation to {description} failed")
        return result

    def run_navigation(self):
        # Go to home position initially
        init_pose = self.create_pose(0.0, 0.0, 0.0)
        self.navigate_with_estop_support(init_pose, "home position (0,0)")

        # Start with first waypoint
        first_waypoint = self.waypoint_manager.get_next_waypoint()
        first_goal_pose = self.create_pose(first_waypoint[0], first_waypoint[1], self.current_yaw)
        self.navigate_with_estop_support(first_goal_pose, f"Initial waypoint {first_waypoint}")

    def distance_to_goal(self, goal_pose):
        current_pose = self.navigator.get_pose()
        if current_pose is None:
            return float('inf')
        dx = current_pose.pose.position.x - goal_pose.pose.position.x
        dy = current_pose.pose.position.y - goal_pose.pose.position.y
        return (dx**2 + dy**2)**0.5

def main():
    rclpy.init()
    robot = RobotNavigator()
    robot.run_navigation()
    rclpy.shutdown()

if __name__ == "__main__":
    main()