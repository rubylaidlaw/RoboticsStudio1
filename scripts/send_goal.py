# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter
# from rclpy.qos import qos_profile_sensor_data
# from nav2_simple_commander.robot_navigator import BasicNavigator
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import OccupancyGrid
# import numpy as np
# import time

# def get_quaternion_from_yaw(yaw):
#     return [0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)]

# def occupancy_grid_to_numpy(msg):
#     data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
#     return data

# def detect_frontiers(occ_grid, resolution, origin):
#     height, width = occ_grid.shape
#     frontiers = []
#     for y in range(1, height - 1):
#         for x in range(1, width - 1):
#             if occ_grid[y, x] == 0:  # free space cell
#                 # If any 8-connected neighbor is unknown, this is a frontier cell
#                 neighbors = occ_grid[y-1:y+2, x-1:x+2].flatten()
#                 if -1 in neighbors:
#                     wx = origin[0] + x * resolution
#                     wy = origin[1] + y * resolution
#                     frontiers.append((wx, wy))
#     return frontiers

# class FrontierExplorer(Node):
#     def __init__(self):
#         super().__init__('frontier_explorer')
#         self.navigator = BasicNavigator()
#         self.declare_parameter('use_sim_time', True)
#         self.map_sub = self.create_subscription(
#             OccupancyGrid,
#             '/map',
#             self.map_callback,
#             qos_profile_sensor_data
#         )
#         self.latest_map = None
#         self.goal_in_progress = False

#     def map_callback(self, msg):
#         self.latest_map = msg

#     def send_to_frontier(self):
#         occ_grid = occupancy_grid_to_numpy(self.latest_map)
#         resolution = self.latest_map.info.resolution
#         origin = (self.latest_map.info.origin.position.x, self.latest_map.info.origin.position.y)

#         robot_pose = self.navigator.getCurrentPose().pose.position
#         rx, ry = robot_pose.x, robot_pose.y

#         # Find all frontiers
#         frontiers = detect_frontiers(occ_grid, resolution, origin)
#         if not frontiers:
#             self.get_logger().info("No more frontiers found. Exploration complete!")
#             return False

#         # Choose the closest frontier
#         distances = [np.hypot(fx - rx, fy - ry) for fx, fy in frontiers]
#         idx = np.argmin(distances)
#         goal_x, goal_y = frontiers[idx]

#         # Set goal pose
#         goal = PoseStamped()
#         goal.header.frame_id = 'map'
#         goal.header.stamp = self.get_clock().now().to_msg()
#         goal.pose.position.x = goal_x
#         goal.pose.position.y = goal_y
#         goal.pose.position.z = 0.0
#         q = get_quaternion_from_yaw(0.0)
#         goal.pose.orientation.x = q[0]
#         goal.pose.orientation.y = q[1]
#         goal.pose.orientation.z = q[2]
#         goal.pose.orientation.w = q[3]

#         self.get_logger().info(f"Navigating to frontier at ({goal_x:.2f}, {goal_y:.2f})")
#         self.navigator.goToPose(goal)
#         self.goal_in_progress = True
#         return True

#     def run(self):
#         while rclpy.ok():
#             rclpy.spin_once(self, timeout_sec=0.5)
#             if self.latest_map is not None and not self.goal_in_progress:
#                 sent = self.send_to_frontier()
#                 if not sent:
#                     break
#             if self.goal_in_progress:
#                 if self.navigator.isTaskComplete():
#                     result = self.navigator.getResult()
#                     self.get_logger().info(f"Arrived at frontier, result: {result}")
#                     self.goal_in_progress = False
#                 else:
#                     continue
#         rclpy.shutdown()

# def main():
#     rclpy.init()
#     explorer = FrontierExplorer()
#     explorer.run()

# if __name__ == '__main__':
#     main()



from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import numpy as np

def get_quaternion_from_yaw(yaw):
    return [0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)]

class move():
    def y_forward(self):
        rclpy.init()
        navigator = BasicNavigator()

        x0, y0 = 0.0, 0.0
        step = 0.5
        max_steps = 100  # 0.1m x 100 = 10m
        yaw = 0.0
        boundary_found = False

        for i in range(max_steps):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x0
            goal_pose.pose.position.y = y0 + (i + 1) * step
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
            print(f"Step {i+1}, Goal: y={goal_pose.pose.position.y:.2f}, Result: {result}")
            # Test for nav failure (could happen at boundary)
            # if not hasattr(result, 'status') or str(result.status) != "SUCCEEDED":
            #     print("Boundary or obstacle detected or unreachable goal! Stopping.")
            #     boundary_found = True
            #     break

        rclpy.shutdown()

    def x_move(self):
        rclpy.init()
        navigator = BasicNavigator()

        x0, y0 = 0.0, 0.0
        step = 0.5
        max_steps = 100  # 0.1m x 100 = 10m
        yaw = 0.0
        boundary_found = False

        for i in range(max_steps):
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x0 + (i + 1) * step
            goal_pose.pose.position.y = y0 
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
            print(f"Step {i+1}, Goal: y={goal_pose.pose.position.y:.2f}, Result: {result}")
            # Test for nav failure (could happen at boundary)
            # if not hasattr(result, 'status') or str(result.status) != "SUCCEEDED":
            #     print("Boundary or obstacle detected or unreachable goal! Stopping.")
            #     boundary_found = True
            #     break

        rclpy.shutdown()

