#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import json
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
import time
from geometry_msgs.msg import Point, Pose
import subprocess
import threading
import random
from ament_index_python.packages import get_package_share_directory
import os
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage

class FoxManagerNode(Node):

    def __init__(self):
        super().__init__('foxManagerNode')
       
        self.declare_parameter('num_foxes', 4)
        self.numFoxes = self.get_parameter('num_foxes').value
        self.get_logger().info(f'NUMMMMMMMMMMMMMMMMMMMMMMM FOXESSSSSSSSSSSSSSS: {self.numFoxes}')

        self.world = "large_demo"
        self.box_name = "fox"
        fox_pkg_share = get_package_share_directory('41068_ignition_bringup')
        self.sdf_path = os.path.join(fox_pkg_share, 'models', 'fox', 'model.sdf')
        self.dead_sdf_path = os.path.join(fox_pkg_share, 'models', 'fox', 'model_dead.sdf')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            TFMessage,
            '/model/husky/pose',
            self.robot_pose_callback,
            10
        )

        self.shot = False
        # Create service client
        self.set_pose_client = self.create_client(SetEntityPose,
                                                 f'/world/{self.world}/set_pose')

        self.spawn_client = self.create_client(SpawnEntity,
                                               f'/world/{self.world}/create')

        self.delete_client = self.create_client(DeleteEntity,
                                                f'/world/{self.world}/remove')

        self.shot_sub = self.create_subscription(Point,
                                                 '/fox_shot',
                                                 self.shootCallback,
                                                 10)

        ## MIN & MAX VARIABLES FOR WORLD
        self.xmin, self.xmax = -10, 10
        self.ymin, self.ymax = -10, 10

        allFoxes = {
            'foxy1': {
                'waypoints': [(2, 0, 0)],
                'current_pos': [2, 0, 0, 0], 
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 4.0,
                'paused': False
            },
            'foxy2': {
                'waypoints': [(3, 3, 0), (3, -5, 0), (3, -5, 0), (-12, 12, 0)],
                'current_pos': [3, 3, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 2.0,
                'paused': False
            },
            'foxy3': {
                'waypoints': [(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)],
                'current_pos': [-10, -10, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 0.0,
                'paused': False
            },
            'foxy4': {
                'waypoints': [(-2, -2, 0), (2, -2, 0), (4, -5, 0), (1, -4, 0)],
                'current_pos': [-2, -2, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 6.0,
                'paused': False
            },
            'foxy5': {
                'waypoints': [(-9, 8, 0), (-5, -5, 0), (8, -5, 0), (2, 2, 0)],
                'current_pos': [-9, 8, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 8.0,
                'paused': False
            },
            'foxy6': {
                'waypoints': [(3, 9, 0), (-4, 3, 0), (2, -1, 0), (-6, -8, 0)],
                'current_pos': [3, 9, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'pause_timer': 10.0,
                'paused': False
            }
        }

        self.foxes = dict(list(allFoxes.items())[:self.numFoxes])

        self.get_logger().info('FoxManager initialized with shot detection')

    def move_all_foxes_step(self, dt=0.05):
        """Move all foxes one step toward their next waypoint"""
        for fox_name, fox_data in self.foxes.items():
            
            fox_data['pause_timer'] += dt

            if not fox_data['paused'] and fox_data['pause_timer'] >= 7.0:

      
                fox_data['paused'] = True
                fox_data['pause_timer'] = 0.0
            
            if fox_data['paused']:

            
                if fox_data['pause_timer'] >= 3.0:
                    fox_data['paused'] = False
                    fox_data['pause_timer'] = 0.0
                else:
                    fox_data['pause_timer'] += dt
                    continue

            wp = fox_data['waypoints'][fox_data['current_wp_index']]
            cx, cy, cz, cyaw = fox_data['current_pos']
            tx, ty, tz = wp
            speed = fox_data['speed']

            # Compute direction
            dx = tx - cx
            dy = ty - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.1:
                fox_data['current_wp_index'] = (fox_data['current_wp_index'] + 1) % len(fox_data['waypoints'])
                continue

            # Move a little toward waypoint
            step = speed * dt
            nx = cx + step * (dx / dist)
            ny = cy + step * (dy / dist)
            nyaw = math.atan2(dy, dx)
            fox_data['current_pos'] = [nx, ny, cz, nyaw]

            # Set pose for that fox (each loop)
            self.set_pose_for_fox(fox_name, nx, ny, cz, nyaw)

    def robot_pose_callback(self, msg):
        """Store robot's position in Gazebo world frame from TFMessage"""
        # TFMessage contains a list of transforms
        # We need to find the one for base_link (or husky)
        for transform in msg.transforms:
            # The transform should be from world to base_link (or similar)
            # Check the child_frame_id to find the robot
            if 'base_link' in transform.child_frame_id or 'husky' in transform.child_frame_id:
                self.robot_world_pose = transform.transform.translation
                
                # Log once for verification
                if not hasattr(self, '_logged_robot_pose'):
                    self.get_logger().info(
                        f"Robot Gazebo world pose: ({self.robot_world_pose.x:.2f}, "
                        f"{self.robot_world_pose.y:.2f}) "
                        f"[frame: {transform.header.frame_id} � {transform.child_frame_id}]"
                    )
                    self._logged_robot_pose = True
                break
    
    def spawnAllFoxes(self):
        """
        Spawn all foxes one by one using the ROS 2 SpawnEntity service.
        Movement will not start until all foxes have been spawned.
        """
        from ros_gz_interfaces.msg import EntityFactory

        for fox_name, data in self.foxes.items():
            # Random spawn position and yaw
            xrand = random.uniform(self.xmin, self.xmax)
            yrand = random.uniform(self.ymin, self.ymax)
            yawrand = random.uniform(-math.pi, math.pi)

            data['current_pos'] = [xrand, yrand, 0.0, yawrand]

            # Convert yaw to quaternion
            qz = math.sin(yawrand / 2)
            qw = math.cos(yawrand / 2)

            # Build the request
            req = SpawnEntity.Request()
            req.entity_factory = EntityFactory()
            req.entity_factory.name = fox_name
            req.entity_factory.sdf_filename = self.sdf_path
            req.entity_factory.pose.position.x = xrand
            req.entity_factory.pose.position.y = yrand
            req.entity_factory.pose.position.z = 0.0
            req.entity_factory.pose.orientation.x = 0.0
            req.entity_factory.pose.orientation.y = 0.0
            req.entity_factory.pose.orientation.z = qz
            req.entity_factory.pose.orientation.w = qw

            # Send async request and wait for completion
            future = self.spawn_client.call_async(req)

        self.get_logger().info("[FoxManager] All foxes spawned successfully!")


    def set_pose_for_fox(self, fox_name, x, y, z, yaw):
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        req = SetEntityPose.Request()
        req.entity.name = fox_name
        req.entity.type = 2  # MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = 0.0
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw
        self.set_pose_client.call_async(req)

    def remove_fox(self, fox_name):
        """Remove a fox entity asynchronously, with a callback."""
        req = DeleteEntity.Request()
        req.entity.name = fox_name
        req.entity.type = 2  # MODEL

        self.get_logger().info(f"[FoxManager] Requesting removal of '{fox_name}'...")

        # Send async request
        self.delete_client.call_async(req)

    def shootCallback(self, msg):
        # Check if we have robot's world pose
        if not hasattr(self, 'robot_world_pose') or self.robot_world_pose is None:
            self.get_logger().warn("Robot world pose not yet available")
            return
        
        # Shot position in odom frame (from msg)
        shot_x_odom = msg.x
        shot_y_odom = msg.y
        
        try:
              # Get robot's current position in odom frame
            trans = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
            robot_x_odom = trans.transform.translation.x
            robot_y_odom = trans.transform.translation.y
            
            # Robot's position in Gazebo world frame (from /model/husky/pose)
            robot_x_world = self.robot_world_pose.x
            robot_y_world = self.robot_world_pose.y
            odom_origin_x = robot_x_world - robot_x_odom
            odom_origin_y = robot_y_world - robot_y_odom
            
            # Transform shot from odom to world frame
            xPose = odom_origin_x + shot_x_odom
            yPose = odom_origin_y + shot_y_odom
            
            self.get_logger().info(f"Shot odom: ({shot_x_odom:.2f}, {shot_y_odom:.2f})")
            self.get_logger().info(f"Robot odom: ({robot_x_odom:.2f}, {robot_y_odom:.2f})")
            self.get_logger().info(f"Robot world: ({robot_x_world:.2f}, {robot_y_world:.2f})")
            self.get_logger().info(f"Odom origin in world: ({odom_origin_x:.2f}, {odom_origin_y:.2f})")
            self.get_logger().warn(f"SHOT WORLD: ({xPose:.2f}, {yPose:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return
        
        # Fox hit detection with distance logging
        closestFox = None
        minDistance = float('inf')
        
        for fox_name, fox_data in self.foxes.items():
            fx, fy, _, _ = fox_data['current_pos']
            distance = math.sqrt((xPose - fx)**2 + (yPose - fy)**2)
            
            self.get_logger().info(f"{fox_name}: world pos ({fx:.2f}, {fy:.2f}), distance = {distance:.2f}m")
            if distance < 1.0 and distance < minDistance:
                closestFox = fox_name
                minDistance = distance
        
        if closestFox:
            self.get_logger().info(f'Shot detected! Hit fox: {closestFox} (distance: {minDistance:.2f}m)')
            self.killFox(closestFox)
        else:
            self.get_logger().info(f'Miss! Closest fox was {minDistance:.2f}m away')

                

    # def shootCallback(self, msg):

  
    #     shot_point = PointStamped()
    #     shot_point.header.frame_id = "odom"  # robot frame
    #     # Use current time; it won't matter because we'll use latest transform
    #     shot_point.header.stamp = rclpy.time.Time().to_msg() 
    #     shot_point.point = msg
    #     self.get_logger().info(f"Looking up transform from {shot_point.header.frame_id} � map")

    #     try:
    #         trans = self.tf_buffer.lookup_transform(
    #             'map', 'odom', rclpy.time.Time()
    #         )
    #         # Get latest available transform, ignoring timestamp
    #         shot_in_map = do_transform_point(shot_point, trans)
    #         frames = self.tf_buffer.all_frames_as_string()
          
    #         self.get_logger().info(f"Transform used: {trans.transform.translation}")

    #         xPose = shot_in_map.point.x + self.robot_world_pose.x
    #         yPose = shot_in_map.point.y + self.robot_world_pose.y
            
    #         self.get_logger().warn(f"X POSE WORLD POINT: {xPose}")
    #         self.get_logger().warn(f"Y POSE WORLD POINT: {yPose}")

    #     except Exception as e:
    #         self.get_logger().warn(f"TF transform failed: {e}")
    #         return
    
            
    #     # Continue with your fox hit detection
    #     closestFox = None
    #     for fox_name, fox_data in self.foxes.items():
    #         fx, fy, _, _ = fox_data['current_pos']
    #         distance = math.sqrt((xPose - fx)**2 + (yPose - fy)**2)
    #         if distance < 1:
    #             closestFox = fox_name

    #     if closestFox:
    #         self.get_logger().info(f'Shot detected! Hit fox: {closestFox}')
    #         self.killFox(closestFox)
    #     else:
    #         self.get_logger().info('Shot detected! But did not hit a fox!')
    #         for name, data in self.foxes.items():
        
    #             self.get_logger().info(f"{name} current_pos: {data['current_pos']}")




    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w).
        Roll, pitch, yaw are in radians.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw

    def killFox(self, hitFox):

        fox = self.foxes[hitFox]
        fox['shot'] = True
        steps = 50

        cx, cy, cz, cyaw = fox['current_pos']

        roll = 0.0
        pitch = math.pi/2
        yaw = cyaw
        z = 0.01
        current_roll = 0.0
        current_pitch = 0.0

        roll_step = (roll - 0.0) / steps
        pitch_step = (pitch - current_pitch) / steps
        z_step = z / steps

        for i in range(1, steps + 1):
            roll = current_roll + roll_step * i
            pitch = current_pitch + pitch_step * i
            cz = cz + z_step * i

            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

            # Send pose
            req = SetEntityPose.Request()
            req.entity.name = hitFox
            req.entity.type = 2  # MODEL
            req.pose.position.x = cx
            req.pose.position.y = cy
            req.pose.position.z = cz
            req.pose.orientation.x = qx
            req.pose.orientation.y = qy
            req.pose.orientation.z = qz
            req.pose.orientation.w = qw
            self.set_pose_client.call_async(req)

            # Update internal state
            fox['current_pos'] = [cx, cy, cz, yaw]

            # Small delay between steps
            time.sleep(0.05)

        
        final_pose = {
            "x": cx,
            "y": cy,
            "z": cz,
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
        }

        dead_name = f"{hitFox}_dead"

        # spawn the white fox at the exact same orientation/pose
        self.spawn_fox_model(dead_name, final_pose)
        self.remove_fox(hitFox)
        time.sleep(5)
        self.remove_fox(dead_name)

        self.get_logger().info(f"{hitFox} has gradually fallen and is now lying down.")



    def set_pose_with_rotation(self, fox_name, x, y, z, roll, pitch, yaw):
        """Set pose with full 3D rotation (roll, pitch, yaw)"""
        # Convert Euler angles to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        req = SetEntityPose.Request()
        req.entity.name = fox_name
        req.entity.type = 2  # MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = z
        req.pose.orientation.x = qx
        req.pose.orientation.y = qy
        req.pose.orientation.z = qz
        req.pose.orientation.w = qw

        self.set_pose_client.call_async(req)

    def spawn_fox_model(self, fox_name, pose):
        """
        Spawn a single fox using ROS 2 SpawnEntity service (through ROS-Ignition bridge),
        following the same pattern as spawnAllFoxes.
        """
        from ros_gz_interfaces.msg import EntityFactory
        from ros_gz_interfaces.srv import SpawnEntity
        import math

        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = fox_name
        req.entity_factory.sdf_filename = self.dead_sdf_path
        req.entity_factory.pose.position.x = pose["x"]
        req.entity_factory.pose.position.y = pose["y"]
        req.entity_factory.pose.position.z = pose["z"]
        req.entity_factory.pose.orientation.x = pose["qx"]
        req.entity_factory.pose.orientation.y = pose["qy"]
        req.entity_factory.pose.orientation.z = pose["qz"]
        req.entity_factory.pose.orientation.w = pose["qw"]

        future = self.spawn_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    manager = FoxManagerNode()

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(manager,), daemon=True)
    spin_thread.start()

    # Give bridge time to start
    time.sleep(3)

    # Spawn all foxes
    manager.spawnAllFoxes()
    time.sleep(2)

    # TEST: Simulate a shot after 5 seconds
    # def test_shot():
    #     time.sleep(5)
    #     # Get foxy1's position and shoot near it
    #     fx, fy, _, _ = manager.foxes['foxy1']['current_pos']

    #     # Publish a shot at foxy1's location
    #     from geometry_msgs.msg import Point
    #     shot_msg = Point()
    #     shot_msg.x = fx   # Slightly offset
    #     shot_msg.y = fy 
    #     shot_msg.z = 1.0  # Detection threshold

    #     manager.get_logger().info(f'TEST: Firing shot at ({shot_msg.x}, {shot_msg.y})')
    #     manager.shootCallback(shot_msg)

    # # Uncomment to test
    # # threading.Thread(target=test_shot, daemon=True).start()
    # threading.Thread(target=test_shot, daemon=True).start()

    # Movement loop
    try:
        while rclpy.ok():
            manager.move_all_foxes_step(dt=0.05)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[FoxManager] Shutting down...")

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()