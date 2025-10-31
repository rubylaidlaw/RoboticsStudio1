#!/usr/bin/env python3
import subprocess
import time
import math

class BoxManager:
    def __init__(self, world="empty", box_name="box", sdf_path=None):
        self.world = world
        self.box_name = box_name
        # Provide the path to your SDF file for the box
        self.sdf_path = sdf_path or "/home/student/ros2_ws/src/RoboticsStudio1/models/box.sdf"
    
    def spawn_box(self, x=0, y=0, z=0):
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
        """Move the box to a new position. yaw in radians."""
        # Convert yaw to quaternion around Z-axis
        qx = 0
        qy = 0
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        
        req = (
            f'name: "{self.box_name}"; '
            f'position: {{x: {x}, y: {y}, z: {z}}}; '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        cmd = [
            "ign", "service", "-s", f"/world/{self.world}/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", req
        ]
        subprocess.run(cmd, capture_output=True)  # Suppress output for smoother animation
    
    def move_smoothly(self, target_x, target_y, target_z=0.5, target_yaw=0, 
                      current_x=0, current_y=0, current_z=0.5, current_yaw=0,
                      duration=2.0, steps=25):
        """
        Move the box smoothly from current position to target position.
        
        Args:
            target_x, target_y, target_z: Target position
            target_yaw: Target orientation (radians)
            current_x, current_y, current_z: Current position
            current_yaw: Current orientation (radians)
            duration: Time in seconds for the movement
            steps: Number of intermediate waypoints
        """
        print(f"[BoxManager] Moving smoothly to ({target_x}, {target_y}, {target_z})...")
        
        sleep_time = duration / steps
        
        for i in range(steps + 1):
            # Linear interpolation for position
            t = i / steps  # 0.0 to 1.0
            x = current_x + (target_x - current_x) * t
            y = current_y + (target_y - current_y) * t
            z = current_z + (target_z - current_z) * t
            
            # Interpolate yaw (handle wrapping around 2*pi)
            yaw_diff = target_yaw - current_yaw
            # Normalize to [-pi, pi]
            while yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            while yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi
            yaw = current_yaw + yaw_diff * t
            
            self.set_pose(x, y, z, yaw)
            
        
        print("[BoxManager] Movement complete.")
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


if __name__ == "__main__":
    manager = BoxManager()
    manager.spawn_box()
    time.sleep(2)
    
    # Track current position
    current_pos = [0, 0, 0, 0]  # x, y, z, yaw
    
    # Move to first position smoothly
    current_pos = manager.move_smoothly(
        target_x=1.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1], 
        current_z=current_pos[2], current_yaw=current_pos[3],
        duration=0.5, steps=25
    )
    
    time.sleep(0.5)
    
    # Move to second position smoothly
    current_pos = manager.move_smoothly(
        target_x=2.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1], 
        current_z=current_pos[2], current_yaw=current_pos[3],
        duration=0.5, steps=25
    )
    
    time.sleep(0.5)
    
    # Move to third position smoothly
    current_pos = manager.move_smoothly(
        target_x=3.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1], 
        current_z=current_pos[2], current_yaw=current_pos[3],
        duration=0.5, steps=25
    )
    
    time.sleep(2)
    manager.remove_box()