#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

class FoxDetector(Node):
    def __init__(self):
        super().__init__('fox_detector')
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None

        # Subscriptions
        self.create_subscription(Image, '/camera/image', self.color_callback, 10)
        self.create_subscription(Image, '/camera/depth/image', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)


        # Publisher
        self.publisher = self.create_publisher(PointStamped, '/fox_target', 10)
        self.marker_pub = self.create_publisher(Marker, '/fox_marker', 10)

        self.get_logger().info(" Fox detector started")

    def info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def color_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red colour range
        lower1 = np.array([0, 70, 50])
        upper1 = np.array([25, 255, 255])  

        mask = cv2.inRange(hsv, lower1, upper1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return

        # Pick largest red patch
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        if w*h < 100:
            return  # ignore small red patches

        # Get depth value at the center of the box
        cx, cy = x + w//2, y + h//2
        depth = self.depth_image[cy, cx]
        if np.isnan(depth) or depth <= 0.1:
            return

        # Publish position in camera frame
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "camera_link"
        target_msg.point.x = float(depth)
        target_msg.point.y = float((cx - self.camera_info.k[2]) * depth / self.camera_info.k[0])
        target_msg.point.z = float((cy - self.camera_info.k[5]) * depth / self.camera_info.k[4])

        self.publisher.publish(target_msg)
        self.publish_marker(target_msg)
        self.get_logger().info(f" Fox detected at ({target_msg.point.x:.2f}, {target_msg.point.y:.2f}, {target_msg.point.z:.2f})")

        #cv2.imshow("Camera Feed", cv_image)
        #cv2.imshow("Red Mask", mask)
        #cv2.waitKey(1)


    def publish_marker(self, point_msg):
        marker = Marker()
        marker.header.frame_id = point_msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fox_detection"
        marker.id = 0
        marker.type = Marker.SPHERE # CROSS shape marker # 10 = cross
        marker.action = Marker.ADD
        marker.pose.position.x = point_msg.point.x
        marker.pose.position.y = point_msg.point.y
        marker.pose.position.z = point_msg.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3  # lasts 3 seconds in RViz
        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = FoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
