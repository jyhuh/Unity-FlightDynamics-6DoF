#!/usr/bin/env python3
# Licensed under the Apache License, Version 2.0.
"""
Unified ROS 2 Humble <-> PX4 SITL <-> Unity LiDAR Bridge for Nav2 Autonomous Navigation.
- Receives 2D Raycast LiDAR scans from Unity over UDP :8890 and publishes /scan.
- Converts PX4 /fmu/out/vehicle_local_position (NED) to ROS 2 TF (map->odom->base_link->laser_frame) and /odom.
- Takes off to hover altitude via PX4 Offboard mode and translates Nav2 /cmd_vel (FLU) into PX4 TrajectorySetpoint.
"""

import math
import os
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def resolve_unity_host() -> str:
    env_host = os.environ.get("UNITY_HOST", "").strip()
    if env_host:
        return env_host
    for name in ("host.lima.internal", "host.docker.internal", "host.containers.internal"):
        try:
            ip = socket.gethostbyname(name)
            if ip and not ip.startswith("127."):
                return ip
        except OSError:
            pass
    return "192.168.5.2"


class UnityNav2Px4Bridge(Node):
    def __init__(self) -> None:
        super().__init__("unity_nav2_px4_bridge")

        self.declare_parameter("takeoff_altitude", 2.5)
        self.declare_parameter("unity_lidar_port", 8890)
        self.takeoff_alt = float(self.get_parameter("takeoff_altitude").value)
        self.unity_host = resolve_unity_host()
        self.unity_port = int(self.get_parameter("unity_lidar_port").value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers (ROS 2 standard)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers (PX4 uXRCE-DDS)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos)

        # Subscribers
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_local_pos, px4_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._on_status, px4_qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        # UDP socket for Unity 2D LiDAR
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.setblocking(False)
        self._last_ping = 0.0

        # State
        self.local_pos: VehicleLocalPosition | None = None
        self.vehicle_status: VehicleStatus | None = None
        self.last_tf_stamp = None
        self.yaw_enu = 0.0
        self.initial_heading_ned: float | None = None
        self.cmd_vel = Twist()
        self.last_cmd_vel_time = 0.0
        self.setpoint_counter = 0
        self.last_arm_attempt = 0.0
        self.hover_reached = False

        # 50 Hz UDP poll for /scan, 20 Hz control loop for PX4 Offboard
        self.create_timer(0.02, self._poll_lidar_udp)
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"Unity Nav2-PX4 Bridge ready (Unity LiDAR: {self.unity_host}:{self.unity_port}, Hover Alt: {self.takeoff_alt:.1f}m)"
        )

    def _publish_static_transforms(self) -> None:
        now = self.get_clock().now().to_msg()
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.rotation.w = 1.0

        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = now
        t_base_laser.header.frame_id = "base_link"
        t_base_laser.child_frame_id = "laser_frame"
        t_base_laser.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([t_map_odom, t_base_laser])

    def _on_status(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    def _on_local_pos(self, msg: VehicleLocalPosition) -> None:
        self.local_pos = msg
        if self.initial_heading_ned is None:
            self.initial_heading_ned = float(msg.heading)

        # NED (N, E, D) -> ENU (E, N, -D)
        x_enu = float(msg.y)
        y_enu = float(msg.x)
        z_enu = float(-msg.z)
        self.yaw_enu = (math.pi * 0.5) - float(msg.heading)

        qz = math.sin(self.yaw_enu * 0.5)
        qw = math.cos(self.yaw_enu * 0.5)
        now = self.get_clock().now().to_msg()

        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = "map"
        t_map_odom.child_frame_id = "odom"
        t_map_odom.transform.rotation.w = 1.0

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = x_enu
        tf_msg.transform.translation.y = y_enu
        tf_msg.transform.translation.z = z_enu
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = now
        t_base_laser.header.frame_id = "base_link"
        t_base_laser.child_frame_id = "laser_frame"
        t_base_laser.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform([t_map_odom, tf_msg, t_base_laser])
        self.last_tf_stamp = now

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x_enu
        odom.pose.pose.position.y = y_enu
        odom.pose.pose.position.z = z_enu
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Body velocity in FLU
        vx_enu = float(msg.vy)
        vy_enu = float(msg.vx)
        c = math.cos(self.yaw_enu)
        s = math.sin(self.yaw_enu)
        odom.twist.twist.linear.x = vx_enu * c + vy_enu * s
        odom.twist.twist.linear.y = -vx_enu * s + vy_enu * c
        odom.twist.twist.linear.z = float(-msg.vz)
        self.odom_pub.publish(odom)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = msg
        self.last_cmd_vel_time = time.monotonic()

    def _poll_lidar_udp(self) -> None:
        now_sec = time.monotonic()
        if now_sec - self._last_ping >= 1.0:
            self._last_ping = now_sec
            try:
                self.udp.sendto(b"SCAN", (self.unity_host, self.unity_port))
            except OSError:
                pass

        while True:
            try:
                data, _ = self.udp.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError:
                break

            if len(data) < 14 or data[:4] != b"SCAN":
                continue

            ray_count, r_min, r_max = struct.unpack_from("<Hff", data, 4)
            expected_len = 14 + ray_count * 4
            if len(data) < expected_len or ray_count < 4:
                continue

            ranges = list(struct.unpack_from(f"<{ray_count}f", data, 14))
            scan = LaserScan()
            scan.header.stamp = self.last_tf_stamp if self.last_tf_stamp is not None else self.get_clock().now().to_msg()
            scan.header.frame_id = "laser_frame"
            scan.angle_min = 0.0
            scan.angle_increment = (2.0 * math.pi) / ray_count
            scan.angle_max = scan.angle_min + scan.angle_increment * (ray_count - 1)
            scan.time_increment = 0.0
            scan.scan_time = 0.1
            scan.range_min = float(r_min)
            scan.range_max = float(r_max)
            scan.ranges = ranges
            self.scan_pub.publish(scan)

    def _send_vehicle_cmd(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _control_loop(self) -> None:
        now_us = int(self.get_clock().now().nanoseconds // 1000)

        # Always stream OffboardControlMode (hybrid Z position + XY velocity)
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = now_us
        mode_msg.position = True
        mode_msg.velocity = True
        self.offboard_mode_pub.publish(mode_msg)

        traj = TrajectorySetpoint()
        traj.timestamp = now_us
        alt_agl = -float(self.local_pos.z) if self.local_pos is not None else 0.0
        if alt_agl >= self.takeoff_alt - 0.35:
            self.hover_reached = True

        # Keep Z at target hover altitude; use Nav2 /cmd_vel for XY + yaw once hovering
        if self.hover_reached and (time.monotonic() - self.last_cmd_vel_time) < 0.5:
            vx_flu = float(self.cmd_vel.linear.x)
            vy_flu = float(self.cmd_vel.linear.y)
            wz_flu = float(self.cmd_vel.angular.z)
            c = math.cos(self.yaw_enu)
            s = math.sin(self.yaw_enu)
            vx_enu = vx_flu * c - vy_flu * s
            vy_enu = vx_flu * s + vy_flu * c
            traj.position = [math.nan, math.nan, -self.takeoff_alt]
            traj.velocity = [vy_enu, vx_enu, math.nan]
            traj.yaw = math.nan
            traj.yawspeed = -wz_flu
        else:
            traj.position = [math.nan, math.nan, -self.takeoff_alt]
            traj.velocity = [0.0, 0.0, math.nan]
            if not self.hover_reached and self.initial_heading_ned is not None:
                traj.yaw = self.initial_heading_ned
                traj.yawspeed = 0.0
            else:
                traj.yaw = math.nan
                traj.yawspeed = 0.0

        self.traj_pub.publish(traj)
        self.setpoint_counter += 1

        if self.setpoint_counter >= 15 and (time.monotonic() - self.last_arm_attempt) >= 2.0:
            armed = self.vehicle_status is not None and self.vehicle_status.arming_state == 2
            offboard = self.vehicle_status is not None and self.vehicle_status.nav_state == 14
            if not offboard:
                self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if not armed:
                self._send_vehicle_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_arm_attempt = time.monotonic()


def main() -> None:
    rclpy.init()
    node = UnityNav2Px4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

