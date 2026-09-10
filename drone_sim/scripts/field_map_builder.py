#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
import json
import os
import sys

from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode


class FieldMapBuilder(Node):
    def __init__(self):
        super().__init__('field_map_builder')

        # Storage for detected world boundary points
        self.raw_points = []
        self.centroid = None
        self.mission_active = False

        # Subscriber for 3D world points from coord_transformer
        self.sub_world_boundary = self.create_subscription(
            PolygonStamped,
            '/boundary/world_coordinates',
            self.boundary_callback,
            10
        )

        # RViz2 Visualization Marker Publisher
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/map/field_markers',
            10
        )

        # PX4 Offboard Control Publishers
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        # Timers: 5Hz for live RViz updates, 10Hz for PX4 Offboard heartbeat
        self.create_timer(0.2, self.timer_publish_live_viz)
        self.create_timer(0.1, self.timer_px4_offboard_control)

        self.get_logger().info("==================================================")
        self.get_logger().info("Field Map Builder & Centroid Autonomous Navigator Ready")
        self.get_logger().info("  1. Continuous map markers publishing to /map/field_markers")
        self.get_logger().info("  2. Press Ctrl+C at any time to save map and launch hover mission")
        self.get_logger().info("==================================================")

    def boundary_callback(self, msg: PolygonStamped):
        """Accumulate field boundary points as the drone flies over tiles."""
        for pt in msg.polygon.points:
            self.raw_points.append([pt.x, pt.y])

        # Continuously update geometric centroid as new points stream in
        if len(self.raw_points) >= 3:
            pts_arr = np.array(self.raw_points)
            c_x = float(np.mean(pts_arr[:, 0]))
            c_y = float(np.mean(pts_arr[:, 1]))
            self.centroid = (c_x, c_y)

    def timer_publish_live_viz(self):
        """Streams green boundary point markers and a red centroid pillar to RViz2."""
        if not self.raw_points:
            return

        marker_array = MarkerArray()

        # 1. Green spheres representing collected field boundary points
        pts_marker = Marker()
        pts_marker.header.frame_id = "map"
        pts_marker.header.stamp = self.get_clock().now().to_msg()
        pts_marker.ns = "field_tiles"
        pts_marker.id = 0
        pts_marker.type = Marker.SPHERE_LIST
        pts_marker.action = Marker.ADD
        pts_marker.scale.x = 0.3
        pts_marker.scale.y = 0.3
        pts_marker.scale.z = 0.3
        pts_marker.color.r = 0.1
        pts_marker.color.g = 0.9
        pts_marker.color.b = 0.2
        pts_marker.color.a = 0.8

        for pt in self.raw_points:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = 0.0
            pts_marker.points.append(p)

        marker_array.markers.append(pts_marker)

        # 2. Red cylinder pillar showing the live geometric centroid
        if self.centroid is not None:
            cent_marker = Marker()
            cent_marker.header.frame_id = "map"
            cent_marker.header.stamp = self.get_clock().now().to_msg()
            cent_marker.ns = "centroid"
            cent_marker.id = 1
            cent_marker.type = Marker.CYLINDER
            cent_marker.action = Marker.ADD
            cent_marker.pose.position.x = self.centroid[0]
            cent_marker.pose.position.y = self.centroid[1]
            cent_marker.pose.position.z = 1.0
            cent_marker.scale.x = 0.8
            cent_marker.scale.y = 0.8
            cent_marker.scale.z = 2.0
            cent_marker.color.r = 1.0
            cent_marker.color.g = 0.1
            cent_marker.color.b = 0.1
            cent_marker.color.a = 1.0
            marker_array.markers.append(cent_marker)

        self.pub_markers.publish(marker_array)

    def timer_px4_offboard_control(self):
        """Continuously sends offboard command once autonomous centroid mission is activated."""
        if self.mission_active and self.centroid is not None:
            timestamp_us = int(self.get_clock().now().nanoseconds / 1000)

            # 1. Offboard control mode heartbeat
            offboard_msg = OffboardControlMode()
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            offboard_msg.timestamp = timestamp_us
            self.pub_offboard_mode.publish(offboard_msg)

            # 2. Position target: [Centroid_North, Centroid_East, -30.0m altitude in PX4 NED]
            traj_msg = TrajectorySetpoint()
            traj_msg.position = [float(self.centroid[0]), float(self.centroid[1]), -30.0]
            traj_msg.yaw = 0.0
            traj_msg.timestamp = timestamp_us
            self.pub_trajectory.publish(traj_msg)

    def compute_centroid_and_save(self, output_filename="field_map.json"):
        """Computes bounding area, saves map file to disk, and triggers navigation."""
        if len(self.raw_points) < 3:
            self.get_logger().error("Cannot create map: Less than 3 boundary points collected!")
            return False

        pts_arr = np.array(self.raw_points)
        x_coords = pts_arr[:, 0]
        y_coords = pts_arr[:, 1]

        c_x = float(np.mean(x_coords))
        c_y = float(np.mean(y_coords))
        self.centroid = (c_x, c_y)

        # Structured JSON field map output
        map_data = {
            "field_boundary_samples_count": int(len(pts_arr)),
            "bounding_box_meters": {
                "min_north": float(np.min(x_coords)),
                "max_north": float(np.max(x_coords)),
                "min_east": float(np.min(y_coords)),
                "max_east": float(np.max(y_coords))
            },
            "centroid_world_coordinates": {
                "north_x_m": c_x,
                "east_y_m": c_y,
                "hover_target_altitude_m": 30.0
            }
        }

        save_path = os.path.expanduser(f"~/drone_ws/{output_filename}")
        with open(save_path, 'w') as f:
            json.dump(map_data, f, indent=4)

        self.get_logger().info(f"Field map saved to: {save_path}")
        self.get_logger().info(f"Field Centroid calculated: North={c_x:.2f}m, East={c_y:.2f}m")
        return True

    def trigger_hover_mission(self):
        """Calculates final centroid, saves map, and flags PX4 offboard control."""
        if self.compute_centroid_and_save():
            self.mission_active = True
            self.get_logger().info(">>> AUTONOMOUS MISSION STARTED: Navigating to 30m altitude over field centroid...")


def main(args=None):
    rclpy.init(args=args)
    node = FieldMapBuilder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Catch Ctrl+C to trigger the centroid calculation mission safely
        node.get_logger().info("Keyboard Interrupt: Finalizing Map and Starting Centroid Hover...")
        node.finalize_map_and_hover()
        
        # Spin safely during the mission phase without throwing invalid context errors
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()