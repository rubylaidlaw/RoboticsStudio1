import rclpy
from rclpy.parameter import Parameter
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf_transformations import euler_from_quaternion  # Make sure ros-humble-tf-transformations is installed
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

def get_quaternion_from_yaw(yaw):
    return [0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)]

def get_robot_pose(tf_buffer):
    printed_msg = False
    while rclpy.ok():
        try:
            print("Frames in TF buffer:", tf_buffer.all_frames_as_string())
            trans = tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception as e:
            if not printed_msg:
                print(f"Waiting for TF frames... {e}")
                printed_msg = True
            time.sleep(0.5)  # reduce from 10 for quicker retries in testing

def main():
    rclpy.init()
    node = rclpy.create_node('move_relative_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    time.sleep(2)  # Wait for TF data stream to start

    navigator = BasicNavigator()
    step = 0.2
    max_steps = 50

    for i in range(max_steps):
        pose = get_robot_pose(tf_buffer)
        if pose is None:
            print("Waiting for robot pose...")
            continue
        x0, y0, yaw = pose
        goal_x = x0 + (i + 1) * step * np.cos(yaw)
        goal_y = y0 + (i + 1) * step * np.sin(yaw)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        q = get_quaternion_from_yaw(yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            rclpy.spin_once(node)

        result = navigator.getResult()
        print(f"Step {i+1}, Goal at ({goal_x:.2f}, {goal_y:.2f}), Result: {result}")

        if not hasattr(result, 'status') or str(result.status) != "SUCCEEDED":
            print("Navigation failed or obstacle detected. Stopping.")
            break

    rclpy.shutdown()

if __name__ == '__main__':
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
