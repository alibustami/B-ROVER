#!/usr/bin/env python3
import numpy as np
import rclpy
from brover_control.msg import Control
from px4_msgs.msg import WheelEncoders
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


class RoverBridge(Node):
    def __init__(self):
        super().__init__("rover_bridge_node")

        self.declare_parameters(
            parameters=[
                ("steer_min_value", -1701),
                ("steer_max_value", 2140),
                ("throttle_scaler_factor", 0.045789),
            ]
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
            .integer_value
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

    def encoders_callback(self, msg):
        timestamp = msg.timestamp
        throttle_value = msg.encoder_position_throttle
        steer_value = msg.encoder_position_steer

        rover_control_msg = Control()

        rover_control_msg.timestamp = timestamp
        rover_control_msg.throttle = (
            float(throttle_value) * self.throttle_scaler_factor
        )
        STEER_MIN_ANGLE = -0.32
        STEER_MAX_ANGLE = 0.2

        rover_control_msg.steer = np.interp(
            steer_value,
            [self.steer_min_value, self.steer_max_value],  # input range
            [-0.32, 0.2],  # output range
        )

        rover_control_msg.steer_angle = np.interp(
            rover_control_msg.steer,
            [STEER_MIN_ANGLE, STEER_MAX_ANGLE],
            [-22.5, 22.5],
        )

        self.rover_control_publisher.publish(rover_control_msg)


def main(args=None):
    rclpy.init(args=args)
    rover_bridge = RoverBridge()
    rclpy.spin(rover_bridge)
    rover_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
