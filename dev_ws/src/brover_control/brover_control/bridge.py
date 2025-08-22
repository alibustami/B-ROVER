#!/usr/bin/env python3
import os
import sys
from math import tan

import numpy as np
import rclpy
from brover_control.msg import Control
from geometry_msgs.msg import Twist
from px4_msgs.msg import WheelEncoders
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from utils import counts_to_deg

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


class RoverBridge(Node):
    def __init__(self):
        super().__init__("rover_bridge_node")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("steer_min_value", -1701),
                ("steer_max_value", 2140),
                ("throttle_scaler_factor", 0.045789),
            ],
        )

        self.steer_min_value = (
            self.get_parameter("steer_min_value")
            .get_parameter_value()
            .integer_value
        )

        self.steer_max_value = (
            self.get_parameter("steer_max_value")
            .get_parameter_value()
            .integer_value
        )

        self.throttle_scaler_factor = (
            self.get_parameter("throttle_scaler_factor")
            .get_parameter_value()
            .double_value
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.rover_control_publisher = self.create_publisher(
            Control, "/rover_control", 20
        )

        self.wheel_encoder_subscriber = self.create_subscription(
            WheelEncoders,
            "/fmu/out/wheel_encoders",
            self.encoders_callback,
            qos_profile,
        )
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)
        self.speed_throttle = 0.0
        self.steer_ticks = 0.0
        self._last_encoder_counts = [0]
        self.last_steer_tick = 0

    def encoders_callback(self, msg):
        self.throttle_ticks = float(msg.encoder_position_throttle)
        self.steer_ticks = float(msg.encoder_position_steer)
        # steer_ticks_raw = float(msg.encoder_position_steer)
        # if steer_ticks_raw != 0:
        #     if steer_ticks_raw % 0 == 2:
        #         if steer_ticks_raw > self.last_steer_tick:
        #             self.steer_ticks += 1
        #         else:
        #             self.steer_ticks -= 1
        #         self.last_steer_tick = steer_ticks_raw
        # else:
        #     self.steer_ticks = 0

        self.steer_angle_deg = counts_to_deg(self.steer_ticks)
        # self.get_logger().info(f"{self.steer_angle_deg = }")
        vx, vy, vz = self.get_px4_encoder_motion()

        self.pub_twist(vx, vy, vz)

    def get_px4_encoder_motion(self):
        encoder_counts = [self.throttle_ticks]

        encoder_offsets = [0]

        encoder_offsets[0] = encoder_counts[0] - self._last_encoder_counts[0]
        self._last_encoder_counts[0] = encoder_counts[0]

        circle_mm = 1005.0  # AKM_CIRCLE_MM - wheel circumference in mm
        circle_pulse = 25.0  # ENCODER_CIRCLE_550 - pulses per wheel revolution
        robot_APB = 455.0  # AKM_WIDTH - half the wheelbase in mm

        speed_mm = [0]
        speed_mm[0] = encoder_offsets[0] * 20 * circle_mm / circle_pulse

        val_vx = speed_mm[0]

        val_vy = self.steer_angle_deg

        steering_rad = val_vy * 3.14159 / 180.0
        if abs(steering_rad) > 0.01:
            val_vz = -val_vx * tan(steering_rad) * 1000 / robot_APB
        else:
            val_vz = 0.0

        return val_vx / 1000.0, val_vy, val_vz  # m/s, degrees, deg/s

    def pub_twist(self, vx, vy, vz):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vz
        self.velPublisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rover_bridge = RoverBridge()
    rclpy.spin(rover_bridge)
    rover_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
