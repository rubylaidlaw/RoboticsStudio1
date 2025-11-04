#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import json
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
import time
from geometry_msgs.msg import Point
import subprocess
import threading
import random

class FoxManagerNode(Node):
    
    def __init__(self):
        super().__init__('foxManagerNode')
        print("&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$4")
        print("&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$4")
        print("&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$4")
       
        self.world = "large_demo"
        self.box_name = "fox"
        self.sdf_path = "/home/student/ros2_ws/src/RoboticsStudio1/models/fox/model.sdf"
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
        self.xmin, self.xmax = -11, 11
        self.ymin, self.ymax = -11, 11

        self.foxes = {
            'foxy1': {
                'waypoints': [(-5, 1, 0), (1, 1, 0), (4, -4, 0), (8, 8, 0)],
                'current_pos': [-5, 1, 0, 0],  # x, y, z, yaw
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'death_animation': None
            },
            'foxy2': {
                'waypoints': [(3, 3, 0), (3, -5, 0), (3, -5, 0), (-12, 12, 0)],
                'current_pos': [3, 3, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'death_animation': None
            },
            'foxy3': {
                'waypoints': [(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)],
                'current_pos': [-10, -10, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'death_animation': None
            },
            'foxy4': {
                'waypoints': [(-2, -2, 0), (2, -2, 0), (4, -5, 0), (1, -4, 0)],
                'current_pos': [-2, -2, 0, 0],
                'current_wp_index': 0,
                'speed': 0.5,
                'shot': False,
                'death_animation': None
            }
        }

        self.get_logger().info('FoxManager initialized with shot detection')
    
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

    
    # def spawnAllFoxes(self):
        
    #     ## SPAWNING ALL FOXES USING ROS BRIDGE AND CREATE_MULTIPLE SERVICE
        
    #     foxes = []
    #     for fox_name, data in self.foxes.items():
            
    #         ## GENERATING RANDOM SPAWN POSE
    #         xrand = random.uniform(self.xmin, self.xmax)
    #         yrand = random.uniform(self.ymin, self.ymax)
    #         yawrand = random.uniform(-math.pi, math.pi)

    #         data['current_pos'] = [xrand, yrand, 0.0, yawrand]

    #         ## QUATERNION FROM YAW
    #         qz = math.sin(yawrand / 2)
    #         qw = math.cos(yawrand / 2)

    #         ## BUILDING THE ENTITY REQUEST 
    #         entity_str = (
    #             f'{{ name: "{fox_name}" '
    #             f'sdf_filename: "{self.sdf_path}" '
    #             f'pose: {{ position: {{ x: {xrand}, y: {yrand}, z: {0.0} }}, '
    #             f'orientation: {{ x: 0, y: 0, z: {qz}, w: {qw} }} }} }}'
    #         )
    #         foxes.append(entity_str)

    #     # WRAPPING THE DATA
    #     req = f'data: [ {", ".join(foxes)} ]'

    #     cmd = [
    #         "ign", "service", "-s", f"/world/{self.world}/create_multiple",
    #         "--reqtype", "ignition.msgs.EntityFactory_V",
    #         "--reptype", "ignition.msgs.Boolean",
    #         "--timeout", "10000",
    #         "--req", req
    #     ]
     
    #     print(f"[FoxManager] Spawning all {len(self.foxes)} foxes...")
    #     result = subprocess.run(cmd, capture_output=True, text=True)

    #     if result.returncode == 0:
    #         print("[FoxManager] All foxes spawned successfully!")
    #         print("Output:", result.stdout)
    #     else:
    #         print("[FoxManager] Error spawning foxes:")
    #         print(result.stderr)

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
        future = self.delete_client.call_async(req)

        # Register callback to handle response later
        def on_delete_done(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f"[FoxManager] '{fox_name}' successfully removed.")
                else:
                    self.get_logger().warn(f"[FoxManager] Failed to remove '{fox_name}' — entity not found?")
            except Exception as e:
                self.get_logger().error(f"[FoxManager] Delete request for '{fox_name}' failed: {e}")

        future.add_done_callback(on_delete_done)

    def shootCallback(self, msg):

        xPose = msg.x
        yPose = msg.y

        closestFox = None

        ## FIND OUT WHCIH FOX WAS SHOT
        for fox_name, fox_data in self.foxes.items():
            fx, fy, _, _ = fox_data['current_pos']
            distance = math.sqrt((xPose - fx)**2 + (yPose - fy)**2)
                
            if distance < 0.1:
            
                closestFox = fox_name
        
        if closestFox:
            self.get_logger().info(f'Shot detected! Hit fox: {closestFox}')
            self.killFox(closestFox)
        else:
            self.get_logger().info(f'Shot detected! But did not hit a fox!')
            for name, data in self.foxes.items():
                self.get_logger().info(f"{name} current_pos: {data['current_pos']}")

            # @TODO ADD IN CALL TO ESTOP HERE
    
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
        
        dead_name = f"{hitFox}_dead"
        self.spawn_fox_model(dead_name, cx, cy, cz, roll, pitch, yaw)

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
        
    def spawn_fox_model(self, fox_name, x, y, z, roll, pitch, yaw):
        """
        Spawn a single fox using ROS 2 SpawnEntity service (through ROS-Ignition bridge),
        following the same pattern as spawnAllFoxes.
        """
        from ros_gz_interfaces.msg import EntityFactory
        from ros_gz_interfaces.srv import SpawnEntity
        import math

        # Convert yaw to quaternion
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        # Build the request
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory()
        req.entity_factory.name = fox_name
        req.entity_factory.sdf_filename = "/home/student/ros2_ws/src/RoboticsStudio1/models/fox/model_dead.sdf"
        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y
        req.entity_factory.pose.position.z = z
        req.entity_factory.pose.orientation.x = -math.pi/2
        req.entity_factory.pose.orientation.y = 0.0
        req.entity_factory.pose.orientation.z = qz
        req.entity_factory.pose.orientation.w = qw

        # Send async request and wait for completion
        future = self.spawn_client.call_async(req)
        # Optionally, wait for the service response
 

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
    def test_shot():
        time.sleep(5)
        # Get foxy1's position and shoot near it
        fx, fy, _, _ = manager.foxes['foxy1']['current_pos']
        
        # Publish a shot at foxy1's location
        from geometry_msgs.msg import Point
        shot_msg = Point()
        shot_msg.x = fx   # Slightly offset
        shot_msg.y = fy 
        shot_msg.z = 1.0  # Detection threshold
        
        manager.get_logger().info(f'TEST: Firing shot at ({shot_msg.x}, {shot_msg.y})')
        manager.shootCallback(shot_msg)
    
    # Uncomment to test
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


if __name__ == '__main__':
    main()