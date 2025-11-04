#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import json
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
import time
import subprocess
import threading
import random

class FoxManagerNode(Node):
    
    def __init__(self):
        super().__init__('foxManagerNode')
        self.world = "large_demo"
        self.box_name = "fox"
        self.sdf_path = "/home/student/ros2_ws/src/RoboticsStudio1/models/fox/model.sdf"
        self.shot = False
        # Create service client
        self.set_pose_client = self.create_client(
            SetEntityPose,
            f'/world/{self.world}/set_pose'
        )

        ## MIN & MAX VARIABLES FOR WORLD
        self.xmin, self.xmax = -11, 11
        self.ymin, self.ymax = -11, 11

        self.foxes = {
            'foxy1': {
                'waypoints': [(-5, 1, 0), (1, 1, 0), (4, -4, 0), (8, 8, 0)],
                'current_pos': [-5, 1, 0, 0],  # x, y, z, yaw
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False
            },
            'foxy2': {
                'waypoints': [(3, 3, 0), (3, -5, 0), (3, -5, 0), (-12, 12, 0)],
                'current_pos': [3, 3, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False
            },
            'foxy3': {
                'waypoints': [(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)],
                'current_pos': [-10, -10, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False
            },
            'foxy4': {
                'waypoints': [(-2, -2, 0), (2, -2, 0), (4, -5, 0), (1, -4, 0)],
                'current_pos': [-2, -2, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False
            }
        }

        self.get_logger().info('BoxManager initialized')
    
    def wait_for_service(self, timeout=10.0):
        """Wait for the set_pose service to be ready"""
        self.get_logger().info('Waiting for set_pose service...')
        start_time = time.time()
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error('Service timeout!')
                return False
            self.get_logger().info('Still waiting...')
        self.get_logger().info('Service is ready!')
        return True
    
    def spawnFox(self, x=0, y=0, z=0.5):

        req = (
            f'name: "{self.box_name}"; '
            f'sdf_filename: "{self.sdf_path}"; '
            f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'
        )

        cmd = [
            "ign", "service", "-s", f"/world/{self.world}/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", req
        ]
        print(f"[BoxManager] Spawning box '{self.box_name}'...")
        subprocess.run(cmd)
        print("[BoxManager] Box spawned.")
    
    
    def spawnAllFoxes(self):
        
        ## SPAWNING ALL FOXES USING ROS BRIDGE AND CREATE_MULTIPLE SERVICE
        
        foxes = []
        for fox_name, data in self.foxes.items():
            
            ## GENERATING RANDOM SPAWN POSE
            xrand = random.uniform(self.xmin, self.xmax)
            yrand = random.uniform(self.ymin, self.ymax)
            yawrand = random.uniform(-math.pi, math.pi)

            data['current_pos'] = [xrand, yrand, 0.0, yawrand]

            ## QUATERNION FROM YAW
            qz = math.sin(yawrand / 2)
            qw = math.cos(yawrand / 2)

            ## BUILDING THE ENTITY REQUEST 
            entity_str = (
                f'{{ name: "{fox_name}" '
                f'sdf_filename: "{self.sdf_path}" '
                f'pose: {{ position: {{ x: {xrand}, y: {yrand}, z: {0.0} }}, '
                f'orientation: {{ x: 0, y: 0, z: {qz}, w: {qw} }} }} }}'
            )
            foxes.append(entity_str)

        # WRAPPING THE DATA
        req = f'data: [ {", ".join(foxes)} ]'

        cmd = [
            "ign", "service", "-s", f"/world/{self.world}/create_multiple",
            "--reqtype", "ignition.msgs.EntityFactory_V",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "10000",
            "--req", req
        ]
        print("************************************* reg: ")
        print("************************************* reg: ", req)
        print("************************************* reg: ")
        print(f"[FoxManager] Spawning all {len(self.foxes)} foxes...")
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode == 0:
            print("[FoxManager] All foxes spawned successfully!")
            print("Output:", result.stdout)
        else:
            print("[FoxManager] Error spawning foxes:")
            print(result.stderr)

    def set_all_poses(self):
        """Send all fox poses to Gazebo using /set_pose_vector service."""
        from ros_gz_interfaces.msg import EntityPose
        from geometry_msgs.msg import Pose

        if not self.set_pose_vector_client.service_is_ready():
            self.get_logger().warn("set_pose_vector service not ready!")
            return

        request = SetEntityPoseVector.Request()

        for name, data in self.foxes.items():
            entity_pose = EntityPose()
            entity_pose.entity.name = name
            entity_pose.entity.type = 2  # MODEL type

            pose = Pose()
            pose.position.x = data['current_pos'][0]
            pose.position.y = data['current_pos'][1]
            pose.position.z = data['current_pos'][2]

            qz = math.sin(data['current_pos'][3] / 2)
            qw = math.cos(data['current_pos'][3] / 2)
            pose.orientation.z = qz
            pose.orientation.w = qw

            entity_pose.pose = pose
            request.entity_poses.entity_poses.append(entity_pose)

        self.set_pose_vector_client.call_async(request)

    def set_pose(self, x=0, y=0, z=0.5, yaw=0):
        """Move the box using ROS 2 service (fast!)"""
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        
        request = SetEntityPose.Request()
        request.entity.name = self.box_name
        request.entity.type = 2  # MODEL type
        
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.x = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = qz
        request.pose.orientation.w = qw
        
        # Call service asynchronously (non-blocking, very fast!)
        self.set_pose_client.call_async(request)
    
    def move_smoothly(self, target_x, target_y, target_z=0.5, target_yaw=0,
                      current_x=0, current_y=0, current_z=0.5, current_yaw=0,
                      speed=1.0, update_rate=50):
        """
        Move the box smoothly with high update rate - NOW IT WILL WORK!
        """
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        duration = distance / speed
        steps = max(10, int(duration * update_rate))
        sleep_time = duration / steps
        
        self.get_logger().info(f'Moving to ({target_x}, {target_y}) at {speed} m/s with {steps} steps @ {update_rate} Hz')
        
        for i in range(steps + 1):
            t = i / steps
            x = current_x + (target_x - current_x) * t
            y = current_y + (target_y - current_y) * t
            z = current_z + (target_z - current_z) * t
            # Interpolate yaw
            yaw_diff = target_yaw - current_yaw
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            yaw = current_yaw + yaw_diff * t
            
            self.set_pose(x, y, z, yaw)
            
            if i < steps:
                time.sleep(sleep_time)
        
        self.get_logger().info('Movement complete!')
        return target_x, target_y, target_z, target_yaw
    
    def move_all_foxes_step(self, dt=0.05):
        """Move all foxes one step toward their next waypoint"""
        for fox_name, fox_data in self.foxes.items():
            if fox_data['shot']:
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

    def remove_box(self):
        """Remove the box using the Ignition remove service"""
        req = f'type: MODEL; name: "{self.box_name}"'
        cmd = [
            "ign", "service", "-s", f"/world/{self.world}/remove",
            "--reqtype", "ignition.msgs.Entity",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", req
        ]
        print(f"[BoxManager] Removing box '{self.box_name}'...")
        subprocess.run(cmd)
        print("[BoxManager] Box removed.")

def main(args=None):
    rclpy.init(args=args)
    
    manager = FoxManagerNode()
    
    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(manager,), daemon=True)
    spin_thread.start()
    
    # Give bridge time to start
    time.sleep(3)
    
    # Spawn the box
    # manager.spawnFox(x=0, y=0, z=0.5)
    manager.spawnAllFoxes()
    time.sleep(2)
    current_pos = [0, 0, 0.5, 0]
    
    # while not manager.shot: 

    #     for pos in manager.waypoints:
            
    #         if manager.shot:
    #             break
            
    #         current_pos = manager.move_smoothly(target_x=pos[0], target_y=pos[1], target_z=0, target_yaw=math.pi/4,
    #                                             current_x=current_pos[0], current_y=current_pos[1],
    #                                             current_z=current_pos[2], current_yaw=current_pos[3],
    #                                             speed=1.0, update_rate=50)
  
    #     time.sleep(3)
  
    # manager.remove_box()
    # After spawning

    # Movement loop
    while rclpy.ok():
        manager.move_all_foxes_step(dt=0.05)
        time.sleep(0.05)


    
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()