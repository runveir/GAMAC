#!/usr/bin/env python3

import os
import sys
import json
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus


INSTRUCTIONS = """
Keyboard teleop (offboard velocity & centroid mission control)
---------------------------------------------------------------
  W / S : forward / backward
  A / D : left / right
  I / K : up / down
  J / L : yaw left / right
  C     : AUTONOMOUS CENTROID MISSION (Go to field centroid & hover at 30m)
  Q     : quit

Any other key or no input -> hover in place (velocity mode).
(Auto-climbs for ~3s after arming before handing over control.)
"""


def load_centroid_target():
    """Reads computed field centroid coordinates from JSON."""
    map_path = os.path.expanduser('~/drone_ws/field_map.json')
    if os.path.exists(map_path):
        try:
            with open(map_path, 'r') as f:
                data = json.load(f)
                centroid = data.get("centroid_world_coordinates", {})
                x = float(centroid.get("north_x_m", 0.0))
                y = float(centroid.get("east_y_m", 0.0))
                alt = float(centroid.get("hover_target_altitude_m", 30.0))
                return x, y, alt
        except Exception as e:
            print(f"Error reading field_map.json: {e}")
    return 0.0, 0.0, 30.0  # Safe fallback default


def get_key(settings, timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    return key


class TeleopOffboard(Node):

    def __init__(self):
        super().__init__('teleop_offboard')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.vehicle_status_callback, qos_profile)

        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()

        # State modes: True = position control (centroid hover), False = velocity control (manual teleop)
        self.centroid_mission_active = False

        # Current commanded velocities (NED frame: +z is down)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Position setpoints for Centroid Mission (NED frame: x=North, y=East, z=-Altitude)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_alt = 30.0

        self.speed = 1.0       # m/s per key press
        self.yaw_speed = 0.5   # rad/s per key press

        self.takeoff_cycles = 30        # ~3s at 10Hz
        self.takeoff_climb_speed = 1.0  # m/s upward during auto-climb

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        if self.centroid_mission_active:
            msg.position = True
            msg.velocity = False
        else:
            msg.position = False
            msg.velocity = True

        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        if self.centroid_mission_active:
            # Position control setpoint (PX4 NED: Z is negative upward)
            msg.position = [float(self.target_x), float(self.target_y), -float(self.target_alt)]
            msg.velocity = [float('nan'), float('nan'), float('nan')]
            msg.yaw = 0.0
        else:
            # Velocity control setpoint
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity = [self.vx, self.vy, self.vz]
            msg.yawspeed = self.yaw_rate

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # Auto-climb window right after arming
        if 10 <= self.offboard_setpoint_counter < 10 + self.takeoff_cycles:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = -self.takeoff_climb_speed
            self.yaw_rate = 0.0

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter < 10 + self.takeoff_cycles + 1:
            self.offboard_setpoint_counter += 1

    def update_velocity_from_key(self, key):
        if self.offboard_setpoint_counter < 10 + self.takeoff_cycles:
            return

        # Trigger Autonomous Centroid Mission on 'c'
        if key.lower() == 'c':
            x, y, alt = load_centroid_target()
            self.target_x = x
            self.target_y = y
            self.target_alt = alt
            self.centroid_mission_active = True
            self.get_logger().info(
                f"AUTONOMOUS MISSION ENGAGED: Navigating to Centroid -> X (North): {x:.2f}m, Y (East): {y:.2f}m, Alt: {alt:.2f}m"
            )
            return

        # Any manual WASD/IJKL key cancels autonomous mode back to manual teleop
        if key in ['w', 's', 'a', 'd', 'i', 'k', 'j', 'l']:
            if self.centroid_mission_active:
                self.centroid_mission_active = False
                self.get_logger().info("Manual override detected. Returned to manual teleop velocity control.")

        # Reset velocities each cycle unless key is held
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        if key == 'w':
            self.vx = self.speed
        elif key == 's':
            self.vx = -self.speed
        elif key == 'a':
            self.vy = -self.speed
        elif key == 'd':
            self.vy = self.speed
        elif key == 'i':
            self.vz = -self.speed
        elif key == 'k':
            self.vz = self.speed
        elif key == 'j':
            self.yaw_rate = -self.yaw_speed
        elif key == 'l':
            self.yaw_rate = self.yaw_speed


def main(args=None):
    rclpy.init(args=args)
    node = TeleopOffboard()

    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print(INSTRUCTIONS)

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)
            if key == 'q':
                break
            node.update_velocity_from_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()