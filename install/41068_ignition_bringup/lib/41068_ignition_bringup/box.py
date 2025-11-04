#!/usr/bin/env python3
import subprocess
import time
import math
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
import threading

class BoxManager(Node):
    def __init__(self, world="empty", box_name="box", sdf_path=None):
        super().__init__('box_manager')
        self.world = world
        self.box_name = box_name
        self.sdf_path = sdf_path or "/home/student/ros2_ws/src/RoboticsStudio1/models/box.sdf"
        
        # Create service client
        self.set_pose_client = self.create_client(
            SetEntityPose,
            f'/world/{self.world}/set_pose'
        )
        
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
    
    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(manager,), daemon=True)
    spin_thread.start()
    
    # Give bridge time to start
    time.sleep(3)
    
    # Spawn the box
    manager.spawn_box(x=0, y=0, z=0.5)
    time.sleep(2)

    # Wait for service to be ready
    if not manager.wait_for_service(timeout=15.0):
        print("ERROR: Service not available")
        manager.destroy_node()
        rclpy.shutdown()
        return
    
    # Track current position
    current_pos = [0, 0, 0.5, 0]
    current_pos = manager.move_smoothly(
        target_x=1.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=50  # 50 updates per second = very smooth!
    )
    time.sleep(0.5)
    
    current_pos = manager.move_smoothly(
        target_x=2.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=50
    )
    time.sleep(0.5)
    
    current_pos = manager.move_smoothly(
        target_x=3.0, target_y=1.0, target_z=0, target_yaw=math.pi/4,
        current_x=current_pos[0], current_y=current_pos[1],
        current_z=current_pos[2], current_yaw=current_pos[3],
        speed=1.0,
        update_rate=50
    )
    time.sleep(2)
    manager.remove_box()
    
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()