from enum import Enum, auto
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np


class NavigationState(Enum):
    MOVE_Y_FORWARD = auto()
    MOVE_X_FORWARD = auto()
    MOVE_Y_BACKWARD = auto()
    MOVE_X_BACKWARD = auto()
    TURN_90_LEFT = auto()
    TURN_90_RIGHT = auto()
    BACKUP = auto()
    STOP = auto()


class RobotNavigator:
    def __init__(self):
        self.state = NavigationState.MOVE_Y_FORWARD
        self.x0, self.y0 = 0.0, 0.0
        self.step = 3.0
        self.step_back = -3.0
        self.max_steps = 100
        self.yaw = 0.0
        self.navigator = None
        self.current_step = 0
        self.turn_angle = math.pi / 2
        self.just_turned = False
        self.just_backed_up = False
        self.current_yaw = 0.0

    def get_quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

    def move_y_forward(self, goal_pose, q):
        goal_pose.pose.position.x = self.x0
        goal_pose.pose.position.y = self.y0 + (self.current_step + 1) * self.step
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print(f"Moving Y forward, Step {self.current_step + 1}")

    def move_x_forward(self, goal_pose, q):
        goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step
        goal_pose.pose.position.y = self.y0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print(f"Moving X forward, Step {self.current_step + 1}")

    def move_y_backward(self, goal_pose, q):
        goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step_back
        goal_pose.pose.position.y = self.y0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print(f"Moving X backward, Step {self.current_step + 1}")

    def move_x_backward(self, goal_pose, q):
        goal_pose.pose.position.x = self.x0
        goal_pose.pose.position.y = self.y0 + (self.current_step + 1) * self.step_back
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print(f"Moving Y backward, Step {self.current_step + 1}")

    def turn_90_left(self, goal_pose, q):
        self.current_yaw += self.turn_angle
        q = self.get_quaternion_from_yaw(self.current_yaw)
        goal_pose.pose.position.x = self.x0
        goal_pose.pose.position.y = self.y0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print("Turning 90 degrees left")

    def turn_90_right(self, goal_pose, q):
        self.current_yaw -= self.turn_angle
        q = self.get_quaternion_from_yaw(self.current_yaw)
        goal_pose.pose.position.x = self.x0
        goal_pose.pose.position.y = self.y0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print("Turning 90 degrees right")
            
    def backup(self, goal_pose, q):
        goal_pose.pose.position.x = self.x0
        goal_pose.pose.position.y = self.y0 + self.step_back
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
        print("Backing up")

    def run_navigation(self, move_y_forward=True, move_x_forward=True, move_y_backward=False, move_x_backward=False, turn_90_left=True, turn_90_right=False, backup=True):
        rclpy.init()
        self.navigator = BasicNavigator()

        while self.state != NavigationState.STOP:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            q = self.get_quaternion_from_yaw(self.current_yaw)

            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator)

            result = self.navigator.getResult()
            print(f"Step {self.current_step + 1}, Result: {result}")

            if result == "SUCCEEDED":
                pass


            if result == "SUCCEEDED":
                self.just_turned = False
                self.just_backed_up = False
                self.current_step += 1
                if self.current_step >= self.max_steps:
                    if self.state == NavigationState.MOVE_Y_FORWARD:
                        self.state = NavigationState.MOVE_X_FORWARD
                    elif self.state == NavigationState.MOVE_X_FORWARD:
                        self.state = NavigationState.STOP
                    self.current_step = 0
            else:
                if not self.just_turned:
                    self.state = NavigationState.TURN_90_LEFT
                    self.just_turned = True
                elif not self.just_backed_up:
                    self.state = NavigationState.BACKUP
                    self.just_backed_up = True
                else:
                    self.state = NavigationState.STOP

        rclpy.shutdown()


def main():
    navigator = RobotNavigator()
    navigator.run_navigation()


if __name__ == "__main__":
    main()