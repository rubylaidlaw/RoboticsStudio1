#!/usr/bin/env python3
import subprocess
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading

class BoxManager(Node):
    def __init__(self, world="empty", box_name="box", sdf_path=None):
        super().__init__('box_manager')
        self.world = world
        self.box_name = box_name
        self.sdf_path = sdf_path or "/home/student/ros2_ws/src/RoboticsStudio1/models/box.sdf"
        
        # Create publisher for setting pose (very fast!)
        self.pose_pub = self.create_publisher(
            Pose,
            f'/model/{self.box_name}/pose',
            10
        )
        
        self.get_logger().info('BoxManager initialized')
    
    def spawn_box(self, x=0, y=0, z=0.5):
        """Spawn the box using the Ignition create service"""
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
    
    def set_pose(self, x=0, y=0, z=0.5, yaw=0):
        """Move the box to a new position using ROS 2 topic (very fast!)"""
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        
        self.pose_pub.publish(pose_msg)
    
    def move_smoothly(self, target_x, target_y, target_z=0.5, target_yaw=0,
                      current_x=0, current_y=0, current_z=0.5, current_yaw=0,
                      speed=1.0, update_rate=30):
        """
        Move the box smoothly - looks like continuous walking motion.
        
        Args:
            target_x, target_y, target_z: Target position
            target_yaw: Target orientation (radians)
            current_x, current_y, current_z: Current position
            current_yaw: Current orientation (radians)
            speed: Movement speed in meters/second
            update_rate: How many updates per second (higher = smoother)
        """
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        duration = distance / speed
        steps = max(5, int(duration * update_rate))
        sleep_time = duration / steps
        
        self.get_logger().info(f'Moving to ({target_x}, {target_y}) at {speed} m/s with {steps} steps')
        
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
        
        self.get_logger().info('Movement complete')
        return target_x, target_y, target_z, target_yaw
    
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
    
    manager = BoxManager()
    
    # Spin in background thread so ROS 2 can handle publishing
    spin_thread = threading.Thread(target=rclpy.spin, args=(manager,), daemon=True)
    spin_thread.start()
    
    # Spawn the box (using ign command)
    manager.spawn_box(x=0, y=0, z=0.5)
    time.sleep(2)
    
    # Track current position
    current_pos = [0, 0, 0.5, 0]  # x, y, z, yaw
    
    # Move smoothly - much faster and smoother now!
    current_pos = manager.move_smoothly(
        target_x=1.0, target_y=1.0, target_z=0.5, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=30  # 30 Hz for very smooth motion
    )
    
    time.sleep(0.5)
    
    # Move to second position
    current_pos = manager.move_smoothly(
        target_x=2.0, target_y=1.0, target_z=0.5, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=30
    )
    
    time.sleep(0.5)
    
    # Move to third position
    current_pos = manager.move_smoothly(
        target_x=3.0, target_y=1.0, target_z=0.5, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=30
    )
    
    time.sleep(2)
    
    # Remove the box (using ign command)
    manager.remove_box()
    
    # Cleanup
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()