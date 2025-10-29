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
        print(f"[BoxManager] Moving box '{self.box_name}' to ({x}, {y}, {z}) yaw={yaw}...")
        subprocess.run(cmd)
        print("[BoxManager] Box moved.")

   
   
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
    manager.set_pose(x=1.0, y=1.0, z=0.0, yaw=math.pi/4)  # rotate 45 degrees
    
    print("[BoxManager] Waiting 5 seconds...")
    time.sleep(2)
    
    manager.set_pose(x=2.0, y=1.0, z=0.0, yaw=math.pi/4) 
    time.sleep(2)
    manager.set_pose(x=3.0, y=1.0, z=0, yaw=math.pi/4) 
    time.sleep(5)

    manager.remove_box()
