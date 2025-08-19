#!/usr/bin/env python3
import numpy as np
import rclpy
from brover_control.msg import Control
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class EncoderMapping(Node):
    def __init__(self):
        super().__init__("encoder_mapping_node")

        self.declare_parameter("wheel_radius", 0.16)
        self.declare_parameter("wheel_base", 0.91)
        self.declare_parameter("track", 0.62)
        self.declare_parameter("max_speed", 2.0)
        self.declare_parameter("steer_min", -0.32)
        self.declare_parameter("steer_max", 0.20)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.L = self.get_parameter("wheel_base").value
        self.W = self.get_parameter("track").value
        self.max_speed = self.get_parameter("max_speed").value
        self.steer_min = self.get_parameter("steer_min").value
        self.steer_max = self.get_parameter("steer_max").value

        self.last_time = self.get_clock().now()

        self.rover_control_subscriber = self.create_subscription(
            Control, "rover_control", self.rover_control_callback, 30
        )
        self.encoder_publisher = self.create_publisher(
            Float32MultiArray, "encoder_values", 30
        )

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def rover_control_callback(self, msg):
        try:
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            if dt <= 0.0 or dt > 0.2:
                return

            # Commands -> Physical
            v = EncoderMapping.clamp(msg.throttle, -1.0, 1.0) * self.max_speed

            s = EncoderMapping.clamp(msg.steer, -1.0, 1.0)

            rack_angle = np.interp(
                s, [-1.0, 1.0], [self.steer_min, self.steer_max]
            )
            rack_angle = EncoderMapping.clamp(
                rack_angle, self.steer_min, self.steer_max
            )

            self.traction_position += (v * dt) / self.wheel_radius
            self.steering_position = rack_angle

            out = Float32MultiArray()
            out.data = [
                float(self.steering_position),
                float(self.traction_position),
            ]
            self.encoder_publisher.publish(out)
        except:
            self.get_logger().error("Failed calculating the positions")


def main(args=None):
    rclpy.init(args=args)
    encoder_mapping = EncoderMapping()
    rclpy.spin(encoder_mapping)
    encoder_mapping.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
