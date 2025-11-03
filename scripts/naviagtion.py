import math
import time
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
import cv2
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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


class NavAndDepthFollower(Node):
    def __init__(self):
        super().__init__('nav_and_depth_follower')

        # --- Navigation ---
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
        self.navigator = BasicNavigator()
        self.current_yaw = 0.0
        self.waypoint_manager = WaypointManager()
        self.estop = Estop()
        self.current_goal = None
        self.nav_active = True

        self.estop_subscription = self.create_subscription(
            Bool, '/estop_status', self.estop_callback, 10
        )

        # --- Depth Following ---
        self.bridge = CvBridge()
        self.latest_depth = None
        self.follow_mode = False  # True, depth follow active. False, nav active.
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_color = self.create_subscription(Image, '/camera/color/image_raw', self.on_color, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, 10)

        # Depth follower params
        self.target_distance = 0.2
        self.max_linear = 0.5
        self.max_angular = 1.5
        self.k_v = 0.8
        self.k_w = 0.003
        self.max_range = 4.0

        self.get_logger().info("NavAndDepthFollower Node initialized")

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

        while not self.navigator.isTaskComplete():
            if self.estop.is_active():
                print("E-Stop activated! Canceling navigation...")
                self.navigator.cancelTask()
                self.estop.wait_if_estop()
                print(f"E-Stop released. Resuming navigation to {description}")
                self.navigator.goToPose(self.current_goal)
            # Switch to follow mode if red is detected
            if self.follow_mode:
                print("Red detected! Cancelling navigation and switching to DepthFollower mode.")
                self.navigator.cancelTask()
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Arrived at {description}")
        elif result == TaskResult.CANCELED:
            print(f"Navigation to {description} was canceled")
        elif result == TaskResult.FAILED:
            print(f"Navigation to {description} failed")
        return result

    def on_depth(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def on_color(self, msg: Image):
        color_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2HSV)
        # --- RED Detection using two HSV ranges ---
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Switch to follow mode if red contour found
        self.follow_mode = False
        bbox = None
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            bbox = cv2.boundingRect(c)
            if bbox[2] * bbox[3] >= 1:  # Sufficiently large ------ change to be bigger than a wombat
                self.follow_mode = True

        if self.follow_mode and self.latest_depth is not None and bbox is not None:
            depth_roi = self.latest_depth[bbox[1]:bbox[1]+bbox[3], bbox[0]:bbox[0]+bbox[2]]
            valid_depths = depth_roi[np.isfinite(depth_roi)]
            twist = Twist()
            if len(valid_depths) == 0:
                self.pub_cmd.publish(twist)
                return
            d = np.median(valid_depths)
            if d > self.max_range or d < 0.1:
                self.pub_cmd.publish(twist)
                return
            height, width = self.latest_depth.shape
            cx = bbox[0] + bbox[2] / 2
            error_ang = cx - width / 2
            error_dist = d - self.target_distance
            v = self.k_v * error_dist
            w = -self.k_w * error_ang
            twist.linear.x = np.clip(v, -self.max_linear, self.max_linear)
            twist.angular.z = np.clip(w, -self.max_angular, self.max_angular)
            self.pub_cmd.publish(twist)
        elif not self.follow_mode:
            twist = Twist()  # stop
            self.pub_cmd.publish(twist)

    def run(self):
        step_limit = 20
        steps_taken = 0
        # Navigate to home first
        init_pose = self.create_pose(0.0, 0.0, 0.0)
        self.navigate_with_estop_support(init_pose, "home position (0,0)")
        # Waypoint navigation loop
        while rclpy.ok() and steps_taken < step_limit:
            # If follow_mode is active, keep DepthFollower mode, pause navigation
            while self.follow_mode and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            # Otherwise, perform navigation
            waypoint = self.waypoint_manager.get_next_waypoint()
            steps_taken += 1
            goal_pose = self.create_pose(waypoint[0], waypoint[1], self.current_yaw)
            self.navigate_with_estop_support(goal_pose, f"waypoint {waypoint}")

def main():
    rclpy.init()
    node = NavAndDepthFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
