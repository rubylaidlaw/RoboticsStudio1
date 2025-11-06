#!/usr/bin/env python3
import rclpy # ROS 2 Python client library 
from rclpy.node import Node # ROS 2 Python Base class for nodes
from geometry_msgs.msg import PoseStamped, PointStamped, Point # Message Types Used (Target Pose)
from visualization_msgs.msg import Marker # For publishing RViz visualisation markers marking hits
from std_msgs.msg import Bool
import math
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class HarpoonLaserNode(Node):

    def __init__(self): # Defines HarpoonFiringNode which inherits from rclpy.node.Node.
        super().__init__('harpoon_laser_node') # Calls the parent constructor, giving the node the name

        # Parameters
        self.declare_parameter('max_range', 10.0) # Declares a ROS2 parameter max_range with default 10 meters.
        self.max_range = float(self.get_parameter('max_range').value) # Reads the parameter into a Python variable self.max_range

        # Subscribe to Fox target from camera detector
        self.target_sub = self.create_subscription(
            PointStamped,
            '/fox_target',
            self.target_callback,
            10
        )

        ## --- Publishers --- ##
        self.hit_pub = self.create_publisher(PoseStamped, '/harpoon/laser_hit', 10) # publishes the hit location of the laser (PoseStamped).
        self.hit_flag_pub = self.create_publisher(Bool, '/harpoon/laser_hit_flag', 10) #  publishes a boolean indicating if a target was hit.
        self.marker_pub = self.create_publisher(Marker, '/harpoon/laser_markers', 10) # publishes RViz markers for laser line and hit sphere.
        # Publish hit positions for FoxManager
        self.fox_shot_pub = self.create_publisher(Point, '/fox_shot', 10)

        self.tf_buffer = Buffer() # holds transforms between frames.
        self.tf_listener = TransformListener(self.tf_buffer, self) # subscribes to TF topics and fills tf_buffer.

        self.get_logger().info('Harpoon laser node started.') # Logs startup message

    
    def target_callback(self, msg: PointStamped):
        # Convert PointStamped → PoseStamped for consistency
        target_pose = PoseStamped()
        target_pose.header = msg.header
        target_pose.pose.position = msg.point

        # Orientation left as identity (no rotation)
        target_pose.pose.orientation.w = 1.0

        # Proceed with same logic as before
        self.process_target(target_pose)
        
    def process_target(self, msg: PoseStamped):

        ## ADDED BY RUBY TO TEST ROS TOPIC EXCHANGE
        self.fox_shot_pub.publish(msg.pose.position)
        
        try:
            trans = self.tf_buffer.lookup_transform(
                'world', 'harpoon_barrel', rclpy.time.Time()
            )
            barrel_x = trans.transform.translation.x
            barrel_y = trans.transform.translation.y
            barrel_z = trans.transform.translation.z
        except Exception as e:
            self.get_logger().warn(f'No TF for harpoon_barrel: {e}')
            return

        tx, ty, tz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        dx, dy, dz = tx - barrel_x, ty - barrel_y, tz - barrel_z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist == 0.0:
            self.get_logger().warn('Target at barrel origin; ignoring')
            return

        if dist > self.max_range:
            scale = self.max_range / dist
            hit_x = barrel_x + dx * scale
            hit_y = barrel_y + dy * scale
            hit_z = barrel_z + dz * scale
            hit = False
        else:
            hit_x, hit_y, hit_z = tx, ty, tz
            hit = True

        hit_flag = Bool()
        hit_flag.data = hit
        self.hit_flag_pub.publish(hit_flag)

        # If the laser hit the target, publish a PoseStamped with the hit location.
        if hit:
            hit_pose = PoseStamped()
            hit_pose.header.frame_id = 'world'
            hit_pose.header.stamp = self.get_clock().now().to_msg()
            hit_pose.pose.position.x = hit_x
            hit_pose.pose.position.y = hit_y
            hit_pose.pose.position.z = hit_z
            hit_pose.pose.orientation = msg.pose.orientation
            self.hit_pub.publish(hit_pose)

            # Publish Point to /fox_shot
            shot_msg = Point()
            shot_msg.x = hit_x
            shot_msg.y = hit_y
            shot_msg.z = hit_z
            self.fox_shot_pub.publish(shot_msg)
            self.get_logger().info(f"Fox Shot at ({hit_x:.2f}, {hit_y:.2f}, {hit_z:.2f})")

        # Laser line marker
        # Publish visualization: laser beam and hit sphere
        # Creates a line strip marker representing the laser beam.
        # Starts at barrel, ends at hit location.
        # Sets color to green, width 2 cm.
        # Published to RViz.
        beam = Marker()
        beam.header.frame_id = 'world'
        beam.header.stamp = self.get_clock().now().to_msg()
        beam.ns = 'harpoon_laser'
        beam.id = 0
        beam.type = Marker.LINE_STRIP
        beam.action = Marker.ADD
        p0 = Point(x=barrel_x, y=barrel_y, z=barrel_z)
        p1 = Point(x=hit_x, y=hit_y, z=hit_z)
        beam.points = [p0, p1]
        beam.scale.x = 0.02
        beam.color.a = 1.0
        beam.color.g = 1.0
        self.marker_pub.publish(beam)

        # Hit sphere marker
        # Publishes a small red sphere at the hit location for RViz visualization.
        hit_marker = Marker()
        hit_marker.header = beam.header
        hit_marker.ns = 'harpoon_laser'
        hit_marker.id = 1
        hit_marker.type = Marker.SPHERE
        hit_marker.action = Marker.ADD
        hit_marker.pose.position.x = hit_x
        hit_marker.pose.position.y = hit_y
        hit_marker.pose.position.z = hit_z
        hit_marker.scale.x = hit_marker.scale.y = hit_marker.scale.z = 0.15
        hit_marker.color.a = 1.0
        hit_marker.color.r = 1.0
        self.marker_pub.publish(hit_marker)

def main(args=None):
    rclpy.init(args=args)
    node = HarpoonLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()        
            
        
        
        
        
        
#         # Callback invoked when a PoseStamped is received on /detected_fox_pose. Logs the target coordinates.
#         def target_callback(self, msg: PoseStamped):
#             try:
#                 trans = self.tf_buffer.lookup_transform('world', 'harpoon_barrel', rclpy.time.Time()) # Looks up the transform from harpoon_barrel to world to get the barrel’s current position.
#                 # Extracts x, y, z of the barrel.
#                 barrel_x = trans.transform.translation.x
#                 barrel_y = trans.transform.translation.y
#                 barrel_z = trans.transform.translation.z
#             except Exception as e:
#                 self.get_logger().warn(f'No TF for harpoon_barrel: {e}') # If the transform is unavailable, logs a warning and returns early.
#                 return
            
#             # Extracts the target coordinates from the PoseStamped message.
#             tx = msg.pose.position.x
#             ty = msg.pose.position.y
#             tz = msg.pose.position.z

#             # Computes the vector from barrel to target (dx, dy, dz).
#             dx = tx - barrel_x
#             dy = ty - barrel_y
#             dz = tz - barrel_z
#             dist = math.sqrt(dx*dx + dy*dy + dz*dz) # Computes Euclidean distance.
#             if dist == 0.0:
#                 self.get_logger().warn('Target at barrel origin; ignoring') # Checks for zero distance to avoid division by zero later.
#                 return

#             # clamp to max_range - If the target is beyond max_range, the laser “hits” at the maximum range along the line.
#             if dist > self.max_range:
#                 scale = self.max_range / dist
#                 hit_x = barrel_x + dx * scale
#                 hit_y = barrel_y + dy * scale
#                 hit_z = barrel_z + dz * scale
#                 hit = False # hit boolean indicates laser hasnt reached target as dist > max_range
#             else:
#                 hit_x = tx
#                 hit_y = ty
#                 hit_z = tz
#                 hit = True # hit boolean indicates laser reached the target and dist to target pose is within range

#             # Optionally: sample occupancy map here along the ray to set hit=False if blocked.

#             # Publish hit flag and pose (only if hit==True)
#             hit_flag = Bool()
#             hit_flag.data = hit
#             self.hit_flag_pub.publish(hit_flag)

#             # If the laser hit the target, publish a PoseStamped with the hit location.
#             if hit:
#                 hit_pose = PoseStamped()
#                 hit_pose.header.frame_id = 'world'
#                 hit_pose.header.stamp = self.get_clock().now().to_msg()
#                 hit_pose.pose.position.x = hit_x
#                 hit_pose.pose.position.y = hit_y
#                 hit_pose.pose.position.z = hit_z
#                 hit_pose.pose.orientation = msg.pose.orientation # Orientation copied from the target (optional, just for visualization).
#                 self.hit_pub.publish(hit_pose)

#             # Publish visualization: laser beam and hit sphere
#             # Creates a line strip marker representing the laser beam.
#             # Starts at barrel, ends at hit location.
#             # Sets color to green, width 2 cm.
#             # Published to RViz.
#             beam = Marker()
#             beam.header.frame_id = 'world'
#             beam.header.stamp = self.get_clock().now().to_msg()
#             beam.ns = 'harpoon_laser'
#             beam.id = 0
#             beam.type = Marker.LINE_STRIP
#             beam.action = Marker.ADD
#             p0 = Point(x=barrel_x, y=barrel_y, z=barrel_z)
#             p1 = Point(x=hit_x, y=hit_y, z=hit_z)
#             beam.points = [p0, p1]
#             beam.scale.x = 0.02  # line width
#             beam.color.a = 1.0
#             beam.color.r = 0.0
#             beam.color.g = 1.0
#             beam.color.b = 0.0
#             self.marker_pub.publish(beam)

#             # Publishes a small red sphere at the hit location for RViz visualization.
#             hit_marker = Marker()
#             hit_marker.header = beam.header
#             hit_marker.ns = 'harpoon_laser'
#             hit_marker.id = 1
#             hit_marker.type = Marker.SPHERE
#             hit_marker.action = Marker.ADD
#             hit_marker.pose.position.x = hit_x
#             hit_marker.pose.position.y = hit_y
#             hit_marker.pose.position.z = hit_z
#             hit_marker.scale.x = 0.15
#             hit_marker.scale.y = 0.15
#             hit_marker.scale.z = 0.15
#             hit_marker.color.a = 1.0
#             hit_marker.color.r = 1.0
#             hit_marker.color.g = 0.0
#             hit_marker.color.b = 0.0
#             self.marker_pub.publish(hit_marker)

#         def main(args=None):
#             rclpy.init(args=args) # Initializes ROS2 Python client library.
#             node = HarpoonLaserNode() # Creates an instance of HarpoonLaserNode.
#             rclpy.spin(node) # Spins the node (keeps it alive to process callbacks).
#             node.destroy_node() # On shutdown, destroys the node 
#             rclpy.shutdown() # Shuts down ROS2 cleanly.


# # Subscribe to Fox Poses recieved from Lidar Sensor 

# # Shoot Laser of Projectile towards Pose Towards Pose
#     # Spawns projectile in barrel of Harpoon
#     # Apply a force or velocity towards the target - using ApplyJointEffor or ApplyBodyWrench in Ignition
#     # OR More simply simulating lazer beam from barrel
#         # ROS2 laser
#         # Use Gazebo/Rignition Raycasting    

# # Mark the pose for where the object was struck