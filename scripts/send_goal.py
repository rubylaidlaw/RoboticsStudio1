from enum import Enum, auto
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np


class NavigationState(Enum):
    MOVE_Y_FORWARD = auto()
    MOVE_X_FORWARD = auto()
    MOVE_Y_BACKWARD = auto()
    MOVE_X_BACKWARD = auto()
    STOP = auto()


class RobotNavigator:
    def __init__(self):
        self.state = NavigationState.MOVE_Y_FORWARD
        self.x0, self.y0 = 0.0, 0.0
        self.step = 2.0
        self.step_back = -2.0
        self.max_steps = 100
        self.yaw = 0.0
        self.navigator = None
        self.current_step = 0

    def get_quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

    def run_navigation(self):
        rclpy.init()
        self.navigator = BasicNavigator()

        while self.state != NavigationState.STOP:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            q = self.get_quaternion_from_yaw(self.yaw)

            if self.state == NavigationState.MOVE_Y_FORWARD:
                goal_pose.pose.position.x = self.x0
                goal_pose.pose.position.y = self.y0 + (self.current_step + 1) * self.step
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                print(f"Moving Y forward, Step {self.current_step+1}")

            elif self.state == NavigationState.MOVE_X_FORWARD:
                goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step
                goal_pose.pose.position.y = self.y0
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                print(f"Moving X forward, Step {self.current_step+1}")

            elif self.state == NavigationState.MOVE_X_BACKWARD:
                goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step_back
                goal_pose.pose.position.y = self.y0
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                print(f"Moving X forward, Step {self.current_step+1}")

            elif self.state == NavigationState.MOVE_Y_BACKWARD:
                goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step_back
                goal_pose.pose.position.y = self.y0
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.x = q[0]
                goal_pose.pose.orientation.y = q[1]
                goal_pose.pose.orientation.z = q[2]
                goal_pose.pose.orientation.w = q[3]

                print(f"Moving X forward, Step {self.current_step+1}")

            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self.navigator)

            result = self.navigator.getResult()
            print(f"Step {self.current_step+1}, Result: {result}")

            if result == "SUCCEEDED":
                self.current_step += 1
                if self.current_step >= self.max_steps:
                    if self.state == NavigationState.MOVE_Y_FORWARD:
                        self.state = NavigationState.MOVE_X_FORWARD
                    elif self.state == NavigationState.MOVE_X_FORWARD:
                        self.state = NavigationState.STOP
                    self.current_step = 0
            else:
                print("Boundary or obstacle detected or unreachable goal! Changing directions.")
                self.current_step += 1
                if self.current_step >= self.max_steps:
                    if self.state == NavigationState.MOVE_Y_FORWARD:
                        self.state = NavigationState.MOVE_X_FORWARD
                    elif self.state == NavigationState.MOVE_X_FORWARD:
                        self.state = NavigationState.STOP
                    self.current_step = 0
                

        rclpy.shutdown()


def main():
    navigator = RobotNavigator()
    navigator.run_navigation()


if __name__ == "__main__":
    main()


# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
# import rclpy
# import numpy as np

# def get_quaternion_from_yaw(yaw):
#     return [0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)]

# rclpy.init()
# navigator = BasicNavigator()

# x0, y0 = 0.0, 0.0
# step = 0.5
# max_steps = 100  # 0.1m x 100 = 10m
# yaw = 0.0
# boundary_found = False

# for i in range(max_steps):
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = 'map'
#     goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#     goal_pose.pose.position.x = x0
#     goal_pose.pose.position.y = y0 + (i + 1) * step
#     goal_pose.pose.position.z = 0.0
#     q = get_quaternion_from_yaw(yaw)
#     goal_pose.pose.orientation.x = q[0]
#     goal_pose.pose.orientation.y = q[1]
#     goal_pose.pose.orientation.z = q[2]
#     goal_pose.pose.orientation.w = q[3]

#     navigator.goToPose(goal_pose)
#     while not navigator.isTaskComplete():
#         rclpy.spin_once(navigator)
#     result = navigator.getResult()
#     print(f"Step {i+1}, Goal: y={goal_pose.pose.position.y:.2f}, Result: {result}")
#     # Test for nav failure (could happen at boundary)
#     # if not hasattr(result, 'status') or str(result.status) != "SUCCEEDED":
#     #     print("Boundary or obstacle detected or unreachable goal! Stopping.")
#     #     boundary_found = True
#     #     break


# rclpy.shutdown()