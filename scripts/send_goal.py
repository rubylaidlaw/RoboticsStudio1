from enum import Enum, auto
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np

# --- Waypoint Manager ---
class WaypointManager:
    def __init__(self, start=(0.0, 0.0), step=3.0):
        self.current_pos = start
        self.step = step
        self.phase = 'y'  # current movement direction phase ('y' or 'x')
        self.phase_count = 0

    def get_next_waypoint(self):
        x, y = self.current_pos
        if self.phase == 'y':
            y += self.step
            self.phase_count += 1
            if self.phase_count >= 3:  # after 3 steps along y, switch to x
                self.phase = 'x'
                self.phase_count = 0
        else:  # self.phase == 'x'
            x += self.step
            self.phase_count += 1
            if self.phase_count >= 2:  # after 2 steps along x, switch back to y
                self.phase = 'y'
                self.phase_count = 0

        self.current_pos = (x, y)
        return self.current_pos

    def reset(self):
        self.current_pos = (0.0, 0.0)
        self.phase = 'y'
        self.phase_count = 0


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

        while steps_taken < step_limit:
            waypoint = self.waypoint_manager.get_next_waypoint()
            steps_taken += 1

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = waypoint[0]
            goal_pose.pose.position.y = waypoint[1]
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



# from enum import Enum, auto
# import math
# import rclpy
# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
# import numpy as np


# class NavigationState(Enum):
#     MOVE_Y_FORWARD = auto()
#     MOVE_X_FORWARD = auto()
#     MOVE_Y_BACKWARD = auto()
#     MOVE_X_BACKWARD = auto()
#     TURN_90_LEFT = auto()
#     TURN_90_RIGHT = auto()
#     BACKUP = auto()
#     STOP = auto()


# class RobotNavigator:
#     def __init__(self):
#         self.state = NavigationState.MOVE_Y_FORWARD
#         self.x0, self.y0 = 0.0, 0.0
#         self.step = 3.0
#         self.step_back = -3.0
#         self.max_steps = 100
#         self.yaw = 0.0
#         self.navigator = None
#         self.current_step = 0
#         self.turn_angle = math.pi / 2
#         self.just_turned = False
#         self.just_backed_up = False
#         self.current_yaw = 0.0

#     def get_quaternion_from_yaw(self, yaw):
#         return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

#     def run_navigation(self):
#         rclpy.init()
#         self.navigator = BasicNavigator()

#         while self.state != NavigationState.STOP:
#             goal_pose = PoseStamped()
#             goal_pose.header.frame_id = 'map'
#             goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

#             q = self.get_quaternion_from_yaw(self.current_yaw)

#             if self.state == NavigationState.MOVE_Y_FORWARD:
#                 goal_pose.pose.position.x = self.x0
#                 goal_pose.pose.position.y = self.y0 + (self.current_step + 1) * self.step
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print(f"Moving Y forward, Step {self.current_step + 1}")

#             elif self.state == NavigationState.MOVE_X_FORWARD:
#                 goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step
#                 goal_pose.pose.position.y = self.y0
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print(f"Moving X forward, Step {self.current_step + 1}")

#             elif self.state == NavigationState.MOVE_X_BACKWARD:
#                 goal_pose.pose.position.x = self.x0 + (self.current_step + 1) * self.step_back
#                 goal_pose.pose.position.y = self.y0
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print(f"Moving X backward, Step {self.current_step + 1}")

#             elif self.state == NavigationState.MOVE_Y_BACKWARD:
#                 goal_pose.pose.position.x = self.x0
#                 goal_pose.pose.position.y = self.y0 + (self.current_step + 1) * self.step_back
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print(f"Moving Y backward, Step {self.current_step + 1}")

#             elif self.state == NavigationState.TURN_90_LEFT:
#                 self.current_yaw += self.turn_angle
#                 q = self.get_quaternion_from_yaw(self.current_yaw)
#                 goal_pose.pose.position.x = self.x0
#                 goal_pose.pose.position.y = self.y0
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print("Turning 90 degrees left")

#             elif self.state == NavigationState.TURN_90_RIGHT:
#                 self.current_yaw -= self.turn_angle
#                 q = self.get_quaternion_from_yaw(self.current_yaw)
#                 goal_pose.pose.position.x = self.x0
#                 goal_pose.pose.position.y = self.y0
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print("Turning 90 degrees right")

#             elif self.state == NavigationState.BACKUP:
#                 goal_pose.pose.position.x = self.x0
#                 goal_pose.pose.position.y = self.y0 + self.step_back
#                 goal_pose.pose.position.z = 0.0
#                 goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = q
#                 print("Backing up")

#             self.navigator.goToPose(goal_pose)
#             while not self.navigator.isTaskComplete():
#                 rclpy.spin_once(self.navigator)

#             result = self.navigator.getResult()
#             print(f"Step {self.current_step + 1}, Result: {result}")

#             if result == "SUCCEEDED":
#                 self.just_turned = False
#                 self.just_backed_up = False
#                 self.current_step += 1
#                 if self.current_step >= self.max_steps:
#                     if self.state == NavigationState.MOVE_Y_FORWARD:
#                         self.state = NavigationState.MOVE_X_FORWARD
#                     elif self.state == NavigationState.MOVE_X_FORWARD:
#                         self.state = NavigationState.STOP
#                     self.current_step = 0
#             else:
#                 if not self.just_turned:
#                     self.state = NavigationState.TURN_90_LEFT
#                     self.just_turned = True
#                 elif not self.just_backed_up:
#                     self.state = NavigationState.BACKUP
#                     self.just_backed_up = True
#                 else:
#                     self.state = NavigationState.STOP

#         rclpy.shutdown()


# def main():
#     navigator = RobotNavigator()
#     navigator.run_navigation()


# if __name__ == "__main__":
#     main()