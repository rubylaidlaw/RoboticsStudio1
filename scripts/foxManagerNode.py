#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import json
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion

        


class FoxManagerNode(Node):
    
    def __init__(self):
        super().__init__('foxManagerNode')
        
        ## SETTING PARAMETERS
        self.foxes = {}
        self.movement_speed = 0.5           # speed of foxes [m/s]
        self.waypoint_threshold = 0.3       # margin of error for waypoints 
        self.world_name = 'large_demo'

        ## PARSE FOX CONFIGS
        fox_configs = self.getConfig()
        try:
            for config in fox_configs:
                fox_id = config['id']
                self.foxes[fox_id] = {
                    'waypoints': config['waypoints'],
                    'current_wp': 0,
                    'alive': True,
                    'pose': list(config['waypoints'][0]) 
                }
        except Exception as e:
            self.get_logger().error(f'Error parsing fox configs: {e}')
        
        ## CREATE SUBSCRIBER FOR KILLED FOXES COMMANDS
        # self.kill_subscriber = self.create_subscription(String,'/fox_shot', self.deadCallback, 10)
        
        ## CREATE TIMER FOR UPDATING MOVEMENTS
        # self.create_timer(0.1, self.updateMovement)
        
        ## CREATE SERVICE SLIENTS 
        self.spawn_client = None        
        self.delete_client = None
        self.pose_client = None
        
        # Spawn initial foxes after a delay
        self.spawn_timer = self.create_timer(2.0, self.spawnInitialFoxes)
        
        self.get_logger().info('Fox Manager Node initialized')
    
    def getConfig(self):
        
        fox_configs = [
                    {'id': '1',
                    'waypoints': [
                    [2.0, 2.0, 0.0],
                    [5.0, 2.0, 0.0],
                    [5.0, 5.0, 0.0],
                    [2.0, 5.0, 0.0]]
                    }, 
                    {'id': '2',
                    'waypoints': [
                    [-2.0, -2.0, 0.0],
                    [-5.0, -2.0, 0.0],
                    [-5.0, -5.0, 0.0],
                    [-2.0, -5.0, 0.0]]
                    }
                ]
        
        return fox_configs


    
    def spawnInitialFoxes(self):
        """Spawn all foxes at their starting positions"""
        from ros_gz_interfaces.srv import SpawnEntity
        
        if self.spawn_client is None:
            self.node = rclpy.create_node('fox')
            self.spawn_client = self.node.create_client(SpawnEntity, '/world/large_demo/create')
        
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Spawn service not available')
            return
        
        for fox_id, data in self.foxes.items():
            if data['alive']:
                self.spawn_fox(fox_id, data['pose'], alive=True)

    def spawn_fox(self, fox_id, position, alive=True):
        """Spawn a fox model at the given position using the correct SpawnEntity interface"""
        
        print("SPAWNING FOX!!!!!")

        request = SpawnEntity.Request()
        request.name = f'fox_{fox_id}'
        request.sdf_filename = 'model.sdf' if alive else 'fox_dead.sdf'
        request.pose = Pose(
            position=Point(x=position[0], y=position[1], z=position[2] if len(position) > 2 else 0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        request.relative_to = "world"

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Spawned fox_{fox_id} ({"alive" if alive else "dead"})')
        )

    
    def delete_fox(self, fox_id):
        """Delete a fox model"""
        
        if self.delete_client is None:
            self.delete_client = self.create_client(DeleteEntity, f'/world/{self.world_name}/remove')
        
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Delete service not available')
            return
        
        request = DeleteEntity.Request()
        request.name = f'fox_{fox_id}'
        
        future = self.delete_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'Deleted fox_{fox_id}')
        )
    
    def deadCallback(self, msg):
        """Handle fox kill messages"""
        fox_id = msg.data
        
        if fox_id not in self.foxes:
            self.get_logger().warn(f'Unknown fox ID: {fox_id}')
            return
        
        if not self.foxes[fox_id]['alive']:
            self.get_logger().warn(f'Fox {fox_id} is already dead')
            return
        
        self.get_logger().info(f'Killing fox {fox_id}')
        self.foxes[fox_id]['alive'] = False
        
        # Get current position before deleting
        current_pose = self.foxes[fox_id]['pose'].copy()
        
        # Delete alive fox and spawn dead fox
        self.delete_fox(fox_id)
        
        # Wait a bit before spawning dead version
        def spawn_dead_fox():
            self.spawn_fox(fox_id, current_pose, alive=False)
            dead_spawn_timer.cancel()
        dead_spawn_timer = self.create_timer(0.5, spawn_dead_fox)
    
    def updateMovement(self):
        """Update fox positions based on waypoints"""
        
        for fox_id, data in self.foxes.items():
            if not data['alive']:
                continue
            
            waypoints = data['waypoints']
            if len(waypoints) == 0:
                continue
            
            current_wp_idx = data['current_wp']
            target = waypoints[current_wp_idx]
            current = data['pose']
            
            # Calculate distance to target
            dx = target[0] - current[0]
            dy = target[1] - current[1]
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if reached waypoint
            if distance < self.waypoint_threshold:
                # Move to next waypoint (loop back to start)
                data['current_wp'] = (current_wp_idx + 1) % len(waypoints)
                continue
            
            # Move towards target
            move_distance = self.movement_speed * 0.1  # timer is 0.1s
            if move_distance > distance:
                move_distance = distance
            
            ratio = move_distance / distance
            new_x = current[0] + dx * ratio
            new_y = current[1] + dy * ratio
            
            # Update stored position
            data['pose'][0] = new_x
            data['pose'][1] = new_y
            
            # Update in Gazebo
            self.set_entity_pose(fox_id, new_x, new_y, current[2] if len(current) > 2 else 0.0)
    
    def set_entity_pose(self, fox_id, x, y, z):
        """Set the pose of an entity in Gazebo"""
       
        # Create client if needed
        if self.pose_client is None:
            self.pose_client = self.create_client(SetEntityPose, f'/world/{self.world_name}/set_pose')
        
        if not self.pose_client.service_is_ready():
            return
        
        request = SetEntityPose.Request()
        request.name = f'fox_{fox_id}'
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        
        # Calculate orientation towards movement direction
        if fox_id in self.foxes:
            waypoints = self.foxes[fox_id]['waypoints']
            current_wp = self.foxes[fox_id]['current_wp']
            if len(waypoints) > 0:
                target = waypoints[current_wp]
                yaw = math.atan2(target[1] - y, target[0] - x)
                request.pose.orientation.z = math.sin(yaw / 2.0)
                request.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pose_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = FoxManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()