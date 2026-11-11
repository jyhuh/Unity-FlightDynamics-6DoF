#!/usr/bin/env python3
# Licensed under the Apache License, Version 2.0.
"""
ROS 2 Humble Offboard & Telemetry Node for Unity FlightDynamics 6-DOF + PX4 SITL.
Communicates with the `px4-sitl` container over Micro-XRCE-DDS (/fmu/in/* and /fmu/out/*).

Usage inside `ros2` container:
  ros2 topic list
  python3 /ros2_ws/unity_px4_offboard_node.py --altitude 10.0 --radius 8.0
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class UnityPx4OffboardNode(Node):
    def __init__(self, target_alt_m: float = 10.0, circle_radius_m: float = 8.0, omega_rad_s: float = 0.35):
        super().__init__("unity_px4_offboard_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers to PX4 (/fmu/in/*)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # Subscribers from PX4 (/fmu/out/*)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_local_pos, qos
        )
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_status, qos
        )

        self.target_alt_ned_z = -abs(target_alt_m)
        self.circle_radius_m = circle_radius_m
        self.omega_rad_s = omega_rad_s

        self.nav_state = 0
        self.arming_state = 0
        self.pos_ned = (0.0, 0.0, 0.0)
        self.counter = 0
        self.phase_angle = 0.0

        # 20 Hz control loop (PX4 requires > 2 Hz OffboardControlMode heartbeat)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self._on_timer)
        self.get_logger().info(
            f"Started ROS 2 Offboard Controller (Target Alt={target_alt_m:.1f}m, Orbit Radius={circle_radius_m:.1f}m)"
        )

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds // 1000)

    def _on_local_pos(self, msg: VehicleLocalPosition) -> None:
        self.pos_ned = (float(msg.x), float(msg.y), float(msg.z))

    def _on_status(self, msg: VehicleStatus) -> None:
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def _on_timer(self) -> None:
        now_us = self._now_us()

        # 1. Publish OffboardControlMode heartbeat
        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position = True
        ocm.velocity = False
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self.offboard_mode_pub.publish(ocm)

        # 2. Switch to OFFBOARD & ARM after 20 setpoint heartbeats (1.0 second)
        if self.counter == 20:
            self.get_logger().info("Requesting PX4 OFFBOARD mode (VEHICLE_CMD_DO_SET_MODE)...")
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Requesting PX4 ARM (VEHICLE_CMD_COMPONENT_ARM_DISARM)...")
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        # 3. Publish TrajectorySetpoint (Climb to altitude first, then fly figure-circle)
        sp = TrajectorySetpoint()
        sp.timestamp = now_us

        current_alt = -self.pos_ned[2]
        target_alt = -self.target_alt_ned_z
        if current_alt < target_alt * 0.80 or self.circle_radius_m <= 0.1:
            sp.position = [0.0, 0.0, float(self.target_alt_ned_z)]
            sp.yaw = 0.0
        else:
            self.phase_angle += self.omega_rad_s * self.dt
            x = self.circle_radius_m * math.cos(self.phase_angle)
            y = self.circle_radius_m * math.sin(self.phase_angle)
            sp.position = [float(x), float(y), float(self.target_alt_ned_z)]
            sp.yaw = float(math.atan2(math.cos(self.phase_angle), -math.sin(self.phase_angle)))

        self.trajectory_pub.publish(sp)

        if self.counter % 40 == 0:
            self.get_logger().info(
                f"[NED Pos] N={self.pos_ned[0]:+.2f}m, E={self.pos_ned[1]:+.2f}m, Alt={-self.pos_ned[2]:+.2f}m"
            )
        self.counter += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Unity 6-DOF + PX4 SITL ROS 2 Offboard Controller")
    parser.add_argument("--altitude", type=float, default=10.0, help="Target hover/orbit altitude in meters")
    parser.add_argument("--radius", type=float, default=8.0, help="Orbit radius in meters (0 for hover)")
    args = parser.parse_args()

    rclpy.init()
    node = UnityPx4OffboardNode(target_alt_m=args.altitude, circle_radius_m=args.radius)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
