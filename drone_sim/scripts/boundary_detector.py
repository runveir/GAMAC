#!/usr/bin/env python3
"""
Detects field boundary coordinates against the environment ground plane
in the downward camera feed using HSV green color filtering.

Publishes:
  /boundary/detected      (geometry_msgs/PolygonStamped) - auto-detected
                           edge points in pixel coordinates
  /boundary/debug_image   (sensor_msgs/Image) - camera feed with detected
                           boundaries overlayed for RQt/RViz viewing

Subscribes:
  /world/drone_world/model/x500_dual_cam_lidar_0/link/camera_down_link/sensor/camera_down/image
  /boundary/manual_override (geometry_msgs/PolygonStamped)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge


class BoundaryDetector(Node):

    def __init__(self):
        super().__init__('boundary_detector')

        self.bridge = CvBridge()
        self.manual_override = None  # set once a manual override arrives

        # HSV Green Threshold Range (Captures Gazebo grass, ignores gray drone legs)
        self.lower_green = np.array([30, 40, 40])
        self.upper_green = np.array([85, 255, 255])
        self.min_contour_area = 300  # pixels

        # Corrected Topic name matching x500_dual_cam_lidar_0 bridge
        self.image_sub = self.create_subscription(
            Image,
            '/world/drone_world/model/x500_dual_cam_lidar_0/link/camera_down_link/sensor/camera_down/image',
            self.image_callback,
            10)

        self.override_sub = self.create_subscription(
            PolygonStamped,
            '/boundary/manual_override',
            self.override_callback,
            10)

        self.boundary_pub = self.create_publisher(
            PolygonStamped, '/boundary/detected', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/boundary/debug_image', 10)

        self.get_logger().info('Boundary detector initialized and listening for downward camera stream.')

    def override_callback(self, msg):
        self.manual_override = msg
        self.get_logger().info('Manual boundary override received.')

    def detect_boundary(self, cv_image):
        """Returns a list of (x, y) pixel coordinates along green field edges."""
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Color mask isolating green field tiles
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        # Morphological noise removal
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        detected_points = []
        for cnt in contours:
            if cv2.contourArea(cnt) >= self.min_contour_area:
                # Downsample contour points (every 5th point) to stream clean boundaries
                for pt in cnt[::5]:
                    u, v = pt[0]
                    detected_points.append((float(u), float(v)))

        return detected_points if detected_points else None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        if self.manual_override is not None:
            boundary_msg = self.manual_override
            corners = [(p.x, p.y) for p in boundary_msg.polygon.points]
        else:
            corners = self.detect_boundary(cv_image)
            if corners is None:
                # Nothing detected this frame; publish unmodified feed to keep rqt active
                debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                return

            boundary_msg = PolygonStamped()
            boundary_msg.header = msg.header
            boundary_msg.polygon.points = [
                Point32(x=float(x), y=float(y), z=0.0) for x, y in corners
            ]

        self.boundary_pub.publish(boundary_msg)

        # Draw green mask boundary overlays for debug view
        debug_image = cv_image.copy()
        for (x, y) in corners:
            cv2.circle(debug_image, (int(x), int(y)), 3, (0, 255, 0), -1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoundaryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()