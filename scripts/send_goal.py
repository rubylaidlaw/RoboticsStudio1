from enum import Enum, auto
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import numpy as np


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
        """Wait (blocking) while estop is active"""
        while self.estop:
            time.sleep(0.1)

    def is_active(self):
        return self.estop


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
            self.index = 0
        wp = self.grid_search_waypoints[self.index]
        self.index += 1
        return wp

    def get_previous_waypoint(self):
        """Get the last waypoint that was retrieved"""
        prev_index = self.index - 1
        if prev_index < 0:
            prev_index = len(self.grid_search_waypoints) - 1
        return self.grid_search_waypoints[prev_index]

    def reset(self):
        self.index = 0


# --- Robot Navigator ---
class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        
        self.navigator = BasicNavigator()
        self.current_yaw = 0.0
        self.waypoint_manager = WaypointManager()
        self.turn_angle = math.pi / 2
        self.estop = Estop()
        self.current_goal = None
        
        # Subscribe to e-stop topic
        self.estop_subscription = self.create_subscription(
            Bool,
            '/estop_status',
            self.estop_callback,
            10
        )
        self.get_logger().info("Robot Navigator initialized, listening for e-stop...")

    def estop_callback(self, msg: Bool):
        """Callback for e-stop status updates"""
        self.estop.set_estop(msg.data)

    def get_quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

    def create_pose(self, x, y, yaw):
        """Helper to create a PoseStamped message"""
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
        """Navigate to a pose with emergency stop support"""
        self.current_goal = goal_pose
        
        print(f"Navigating to {description}")
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            # Check for emergency stop
            if self.estop.is_active():
                print("E-Stop activated! Canceling navigation...")
                self.navigator.cancelTask()
                
                # Wait until estop is released
                self.estop.wait_if_estop()
                
                # Resume navigation to the same goal
                print(f"E-Stop released. Resuming navigation to {description}")
                self.navigator.goToPose(self.current_goal)
            
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
        step_limit = 20
        steps_taken = 0

        # Navigate to home position
        init_pose = self.create_pose(0.0, 0.0, 0.0)
        self.navigate_with_estop_support(init_pose, "home position (0,0)")

        # Navigate through waypoints
        while steps_taken < step_limit:
            waypoint = self.waypoint_manager.get_next_waypoint()
            steps_taken += 1

            goal_pose = self.create_pose(waypoint[0], waypoint[1], self.current_yaw)
            self.navigate_with_estop_support(goal_pose, f"waypoint {waypoint}")


def main():
    rclpy.init()
    robot = RobotNavigator()
    robot.run_navigation()
    rclpy.shutdown()


if __name__ == "__main__":
    main()