import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, Float32
import numpy as np
import random
from geometry_msgs.msg import PoseWithCovarianceStamped


class Estop:
    def __init__(self):
        self.active = False

    def set_estop(self, status: bool):
        if status != self.active:
            self.active = status
            msg = "Emergency Stop enabled! Robot motions will pause." if status else "Emergency Stop released! Resuming robot motions."
            print(msg)

    def wait_if_active(self):
        while self.active:
            time.sleep(0.1)

    def is_active(self):
        return self.active


class WaypointManager:
    def __init__(self, x_bounds=(-10, 10), y_bounds=(-10, 10)):
        self.x_min, self.x_max = x_bounds
        self.y_min, self.y_max = y_bounds
        self.generated_waypoints = []

    def get_next_waypoint(self):
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        waypoint = (x, y)
        self.generated_waypoints.append(waypoint)
        return waypoint


class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')

        self.navigator = BasicNavigator()
        self.current_yaw = 0.0
        self.waypoint_manager = WaypointManager()
        self.estop = Estop()
        self.current_goal = None
        self.current_fox_goal = None
        self.fox_detected = False
        self.fox_target_point = None
        self.current_pose = None
        self.current_random_waypoint = None

        self.last_fox_target_time = 0
        self.fox_target_timeout = 2.0  # seconds before losing fox detection

        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

        self.estop_subscription = self.create_subscription(Bool, '/estop_status', self.estop_callback, 10)
        self.fox_target_subscription = self.create_subscription(PointStamped, '/fox_target', self.fox_target_callback, 10)

        self.distance_pub = self.create_publisher(Float32, '/distance_to_goal', 10)
        self.get_logger().info("Robot Navigator initialized, listening to e-stop and fox target...")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def estop_callback(self, msg: Bool):
        self.estop.set_estop(msg.data)

    def fox_target_callback(self, msg: PointStamped):
        self.fox_target_point = msg
        self.fox_detected = True
        self.last_fox_target_time = time.time()
        # Clear random waypoint when fox detected
        self.current_random_waypoint = None
        self.current_goal = None

    def check_fox_timeout(self):
        if self.fox_detected and (time.time() - self.last_fox_target_time) > self.fox_target_timeout:
            self.get_logger().info("Fox target lost due to timeout, resuming random waypoint navigation")
            self.fox_detected = False
            self.current_fox_goal = None
            self.current_goal = None
            self.current_random_waypoint = None

    @staticmethod
    def get_quaternion_from_yaw(yaw):
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

    def distance_to_goal(self, goal_pose):
        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - goal_pose.pose.position.x
        dy = self.current_pose.position.y - goal_pose.pose.position.y
        return (dx ** 2 + dy ** 2) ** 0.5

    def navigate_to_goal(self, goal_pose, description="", arrival_threshold=2.0):
        self.current_goal = goal_pose
        self.navigator.goToPose(goal_pose)
        last_print_time = 0

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            dist = feedback.distance_remaining if feedback and hasattr(feedback, "distance_remaining") else self.distance_to_goal(self.current_goal)
            self.distance_pub.publish(Float32(data=dist))

            current_time = time.time()
            if current_time - last_print_time > 1.0:
                last_print_time = current_time

            if dist < arrival_threshold:
                return True  # Early return when within threshold distance

            if self.estop.is_active():
                print("E-Stop activated! Canceling navigation...")
                self.navigator.cancelTask()
                self.estop.wait_if_active()
                print(f"E-Stop released. Resuming navigation to {description}")
                self.navigator.goToPose(self.current_goal)

            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Arrived at {description}")
            return True
        else:
            print(f"Navigation to {description} ended with status {result}")
            return False

    @staticmethod
    def positions_are_close(p1, p2, threshold=0.1):
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        return (dx * dx + dy * dy) ** 0.5 < threshold

    def get_goal_offset_from_fox(self, fox_x, fox_y, stop_distance=2.0):
        if self.current_pose is None:
            return fox_x, fox_y

        rx = self.current_pose.position.x
        ry = self.current_pose.position.y

        dx = fox_x - rx
        dy = fox_y - ry
        distance = (dx ** 2 + dy ** 2) ** 0.5

        if distance <= stop_distance:
            return rx, ry

        ratio = (distance - stop_distance) / distance
        goal_x = rx + dx * ratio
        goal_y = ry + dy * ratio
        return goal_x, goal_y

    def run_navigation(self):
        # Go to home position first
        init_pose = self.create_pose(0.0, 0.0, 0.0)
        self.navigate_to_goal(init_pose, "home position (0,0)")

        while rclpy.ok():
            self.check_fox_timeout()

            # If fox detected, lock onto and follow fox
            if self.fox_detected and self.fox_target_point:
                x, y = self.fox_target_point.point.x, self.fox_target_point.point.y

                # Update or set new fox goal if moved significantly
                if (self.current_fox_goal is None or
                    not self.positions_are_close(
                        (x, y),
                        (self.current_fox_goal.pose.position.x, self.current_fox_goal.pose.position.y),
                        threshold=0.1)):

                    goal_x, goal_y = self.get_goal_offset_from_fox(x, y, stop_distance=2.0)

                    fox_pose = self.create_pose(goal_x, goal_y, self.current_yaw)

                    # Cancel existing navigation if running
                    if self.current_goal and not self.navigator.isTaskComplete():
                        self.get_logger().info("Updating fox target - cancelling current navigation...")
                        self.navigator.cancelTask()
                        time.sleep(0.2)  # Allow cancel to process

                    self.get_logger().info(f"Locking onto fox target at ({goal_x:.2f}, {goal_y:.2f})")
                    self.navigator.goToPose(fox_pose)
                    self.current_fox_goal = fox_pose
                    self.current_goal = fox_pose

                else:
                    self.get_logger().info(f"Following fox at ({self.current_fox_goal.pose.position.x:.2f}, {self.current_fox_goal.pose.position.y:.2f})")

                # Check if fox navigation task complete or within 2m
                if self.navigator.isTaskComplete() or \
                   self.distance_to_goal(self.current_fox_goal) < 2.0:
                    self.get_logger().info("Arrived near fox target position, resuming waypoint navigation")
                    self.fox_detected = False
                    self.current_fox_goal = None
                    self.current_goal = None
                    self.current_random_waypoint = None 

            else:
                if (self.current_random_waypoint is None or
                    (self.current_pose is not None and
                     self.distance_to_goal(self.create_pose(self.current_random_waypoint[0], self.current_random_waypoint[1], self.current_yaw)) < 2.0)):
                    self.current_random_waypoint = self.waypoint_manager.get_next_waypoint()
                    self.get_logger().info(f"New random waypoint: {self.current_random_waypoint}")

                waypoint_pose = self.create_pose(self.current_random_waypoint[0], self.current_random_waypoint[1], self.current_yaw)

                if (self.current_goal is None or
                    not self.positions_are_close((self.current_random_waypoint[0], self.current_random_waypoint[1]),
                                                (self.current_goal.pose.position.x, self.current_goal.pose.position.y),
                                                threshold=0.1)):
                    self.get_logger().info(f"Navigating to random waypoint at ({self.current_random_waypoint[0]:.2f}, {self.current_random_waypoint[1]:.2f})")
                    self.navigate_to_goal(waypoint_pose, f"waypoint {(self.current_random_waypoint[0], self.current_random_waypoint[1])}")
                    self.current_fox_goal = None
                    self.current_goal = waypoint_pose

            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    robot = RobotNavigator()
    robot.run_navigation()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# import time
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, PointStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from std_msgs.msg import Bool, Float32
# import numpy as np
# import random
# from geometry_msgs.msg import PoseWithCovarianceStamped


# class Estop:
#     def __init__(self):
#         self.active = False

#     def set_estop(self, status: bool):
#         if status != self.active:
#             self.active = status
#             msg = "Emergency Stop enabled! Robot motions will pause." if status else "Emergency Stop released! Resuming robot motions."
#             print(msg)

#     def wait_if_active(self):
#         while self.active:
#             time.sleep(0.1)

#     def is_active(self):
#         return self.active


# class WaypointManager:
#     def __init__(self, x_bounds=(-10, 10), y_bounds=(-10, 10)):
#         self.x_min, self.x_max = x_bounds
#         self.y_min, self.y_max = y_bounds
#         self.generated_waypoints = []

#     def get_next_waypoint(self):
#         x = random.uniform(self.x_min, self.x_max)
#         y = random.uniform(self.y_min, self.y_max)
#         waypoint = (x, y)
#         self.generated_waypoints.append(waypoint)
#         return waypoint


# class RobotNavigator(Node):
#     def __init__(self):
#         super().__init__('robot_navigator')

#         self.navigator = BasicNavigator()
#         self.current_yaw = 0.0
#         self.waypoint_manager = WaypointManager()
#         self.estop = Estop()
#         self.current_goal = None
#         self.current_fox_goal = None
#         self.fox_detected = False
#         self.fox_target_point = None
#         self.current_pose = None
#         self.current_random_waypoint = None

#         self.pose_subscription = self.create_subscription(
#             PoseWithCovarianceStamped,
#             '/amcl_pose',
#             self.pose_callback,
#             10)

#         self.estop_subscription = self.create_subscription(Bool, '/estop_status', self.estop_callback, 10)
#         self.fox_target_subscription = self.create_subscription(PointStamped, '/fox_target', self.fox_target_callback, 10)

#         self.distance_pub = self.create_publisher(Float32, '/distance_to_goal', 10)
#         self.get_logger().info("Robot Navigator initialized, listening to e-stop and fox target...")

#     # Removed clamp function

#     def pose_callback(self, msg: PoseWithCovarianceStamped):
#         self.current_pose = msg.pose.pose

#     def estop_callback(self, msg: Bool):
#         self.estop.set_estop(msg.data)

#     def fox_target_callback(self, msg: PointStamped):
#         self.fox_target_point = msg
#         self.fox_detected = True
#         # Clear random waypoint when fox detected
#         self.current_random_waypoint = None
#         self.current_goal = None

#     @staticmethod
#     def get_quaternion_from_yaw(yaw):
#         return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

#     def create_pose(self, x, y, yaw):
#         pose = PoseStamped()
#         pose.header.frame_id = 'map'
#         pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         pose.pose.position.x = float(x)
#         pose.pose.position.y = float(y)
#         pose.pose.position.z = 0.0
#         q = self.get_quaternion_from_yaw(yaw)
#         pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
#         return pose

#     def distance_to_goal(self, goal_pose):
#         if self.current_pose is None:
#             return float('inf')
#         dx = self.current_pose.position.x - goal_pose.pose.position.x
#         dy = self.current_pose.position.y - goal_pose.pose.position.y
#         return (dx ** 2 + dy ** 2) ** 0.5

#     def navigate_to_goal(self, goal_pose, description="", arrival_threshold=2.0):
#         self.current_goal = goal_pose
#         self.navigator.goToPose(goal_pose)
#         last_print_time = 0

#         while not self.navigator.isTaskComplete():
#             feedback = self.navigator.getFeedback()
#             dist = feedback.distance_remaining if feedback and hasattr(feedback, "distance_remaining") else self.distance_to_goal(self.current_goal)
#             self.distance_pub.publish(Float32(data=dist))

#             current_time = time.time()
#             if current_time - last_print_time > 1.0:
#                 last_print_time = current_time

#             if dist < arrival_threshold:
#                 return True  # Early return when within threshold distance

#             if self.estop.is_active():
#                 print("E-Stop activated! Canceling navigation...")
#                 self.navigator.cancelTask()
#                 self.estop.wait_if_active()
#                 print(f"E-Stop released. Resuming navigation to {description}")
#                 self.navigator.goToPose(self.current_goal)

#             rclpy.spin_once(self, timeout_sec=0.1)

#         result = self.navigator.getResult()
#         if result == TaskResult.SUCCEEDED:
#             print(f"Arrived at {description}")
#             return True
#         else:
#             print(f"Navigation to {description} ended with status {result}")
#             return False


#     @staticmethod
#     def positions_are_close(p1, p2, threshold=0.1):
#         dx = p1[0] - p2[0]
#         dy = p1[1] - p2[1]
#         return (dx * dx + dy * dy) ** 0.5 < threshold

#     def get_goal_offset_from_fox(self, fox_x, fox_y, stop_distance=2.0):
#         if self.current_pose is None:
#             return fox_x, fox_y

#         rx = self.current_pose.position.x
#         ry = self.current_pose.position.y

#         dx = fox_x - rx
#         dy = fox_y - ry
#         distance = (dx ** 2 + dy ** 2) ** 0.5

#         if distance <= stop_distance:
#             return rx, ry

#         ratio = (distance - stop_distance) / distance
#         goal_x = rx + dx * ratio
#         goal_y = ry + dy * ratio
#         return goal_x, goal_y

#     def run_navigation(self):
#         # Go to home position first
#         init_pose = self.create_pose(0.0, 0.0, 0.0)
#         self.navigate_to_goal(init_pose, "home position (0,0)")

#         while rclpy.ok():
#             # If fox detected, lock onto and follow fox
#             if self.fox_detected and self.fox_target_point:
#                 x, y = self.fox_target_point.point.x, self.fox_target_point.point.y

#                 # Update or set new fox goal if moved significantly
#                 if (self.current_fox_goal is None or
#                     not self.positions_are_close(
#                         (x, y),
#                         (self.current_fox_goal.pose.position.x, self.current_fox_goal.pose.position.y),
#                         threshold=0.1)):

#                     goal_x, goal_y = self.get_goal_offset_from_fox(x, y, stop_distance=2.0)

#                     fox_pose = self.create_pose(goal_x, goal_y, self.current_yaw)

#                     # Cancel existing navigation if running
#                     if self.current_goal and not self.navigator.isTaskComplete():
#                         self.get_logger().info("Updating fox target - cancelling current navigation...")
#                         self.navigator.cancelTask()
#                         time.sleep(0.2)  # Allow cancel to process

#                     self.get_logger().info(f"Locking onto fox target at ({goal_x:.2f}, {goal_y:.2f})")
#                     self.navigator.goToPose(fox_pose)
#                     self.current_fox_goal = fox_pose
#                     self.current_goal = fox_pose

#                 else:
#                     self.get_logger().info(f"Following fox at ({self.current_fox_goal.pose.position.x:.2f}, {self.current_fox_goal.pose.position.y:.2f})")

#                 # Check if fox navigation task complete
#                 if self.navigator.isTaskComplete():
#                     result = self.navigator.getResult()
#                     if result == TaskResult.SUCCEEDED:
#                         self.get_logger().info("Arrived at fox target position, resuming waypoint navigation")
#                         self.fox_detected = False
#                         self.current_fox_goal = None
#                         self.current_goal = None
#                         self.current_random_waypoint = None  # reset random waypoints to pick a new one

#             else:
#                 # No fox detected: navigate random waypoints with 2m proximity logic
#                 if (self.current_random_waypoint is None or
#                     (self.current_pose is not None and
#                      self.distance_to_goal(self.create_pose(self.current_random_waypoint[0], self.current_random_waypoint[1], self.current_yaw)) < 2.0)):
#                     self.current_random_waypoint = self.waypoint_manager.get_next_waypoint()
#                     self.get_logger().info(f"New random waypoint: {self.current_random_waypoint}")

#                 waypoint_pose = self.create_pose(self.current_random_waypoint[0], self.current_random_waypoint[1], self.current_yaw)

#                 if (self.current_goal is None or
#                     not self.positions_are_close((self.current_random_waypoint[0], self.current_random_waypoint[1]),
#                                                 (self.current_goal.pose.position.x, self.current_goal.pose.position.y),
#                                                 threshold=0.1)):
#                     self.get_logger().info(f"Navigating to random waypoint at ({self.current_random_waypoint[0]:.2f}, {self.current_random_waypoint[1]:.2f})")
#                     self.navigate_to_goal(waypoint_pose, f"waypoint {(self.current_random_waypoint[0], self.current_random_waypoint[1])}")
#                     self.current_fox_goal = None
#                     self.current_goal = waypoint_pose

#             rclpy.spin_once(self, timeout_sec=0.1)

# def main():
#     rclpy.init()
#     robot = RobotNavigator()
#     robot.run_navigation()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()