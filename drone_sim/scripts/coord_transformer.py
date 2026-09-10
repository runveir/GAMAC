#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from geometry_msgs.msg import PolygonStamped, Point32
from px4_msgs.msg import VehicleOdometry


class CoordTransformer(Node):
    def __init__(self):
        super().__init__('coord_transformer')

        self.drone_pose = [0.0, 0.0, 1.0]

        # Best Effort QoS required for PX4 uORB topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_odom = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            sensor_qos
        )
        self.sub_pixels = self.create_subscription(
            PolygonStamped,
            '/boundary/detected',
            self.pixel_callback,
            10
        )

        self.pub_world_coords = self.create_publisher(
            PolygonStamped,
            '/boundary/world_coordinates',
            10
        )

        self.get_logger().info("Coordinate Transformer active and synced with PX4 QoS.")

    def odom_callback(self, msg: VehicleOdometry):
        self.drone_pose = [
            float(msg.position[1]),   # East -> ROS X
            float(msg.position[0]),   # North -> ROS Y
            abs(float(msg.position[2]))  # Altitude Z
        ]

    def pixel_callback(self, msg: PolygonStamped):
        pts_count = len(msg.polygon.points)
        if pts_count == 0:
            return

        img_w, img_h = 640.0, 480.0
        fov_h = np.radians(80.0)
        alt = max(self.drone_pose[2], 0.5)

        world_msg = PolygonStamped()
        world_msg.header.frame_id = "map"
        world_msg.header.stamp = self.get_clock().now().to_msg()

        ground_scale = 2.0 * alt * np.tan(fov_h / 2.0) / img_w

        for pt in msg.polygon.points:
            rel_x = (pt.x - (img_w / 2.0)) * ground_scale
            rel_y = (pt.y - (img_h / 2.0)) * ground_scale

            world_x = self.drone_pose[0] + rel_x
            world_y = self.drone_pose[1] - rel_y

            p = Point32()
            p.x = float(world_x)
            p.y = float(world_y)
            p.z = 0.0
            world_msg.polygon.points.append(p)

        self.pub_world_coords.publish(world_msg)
        self.get_logger().info(
            f"Published {len(world_msg.polygon.points)} 3D points to /boundary/world_coordinates",
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformer()
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