#!/usr/bin/env python3
from math import cos, pi, sin, tan

import rclpy
import tf_transformations
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        # Declare parameters
        self.declare_parameter("wheelbase", 0.91)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_footprint_frame", "base_footprint")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("linear_scale_x", 1.0)
        self.declare_parameter("linear_scale_y", 1.0)
        self.declare_parameter("pub_odom_tf", True)

        # Get parameters
        self.wheelbase = (
            self.get_parameter("wheelbase").get_parameter_value().double_value
        )
        self.odom_frame = (
            self.get_parameter("odom_frame").get_parameter_value().string_value
        )
        self.base_footprint_frame = (
            self.get_parameter("base_footprint_frame")
            .get_parameter_value()
            .string_value
        )
        self.base_link_frame = (
            self.get_parameter("base_link_frame")
            .get_parameter_value()
            .string_value
        )
        self.linear_scale_x = (
            self.get_parameter("linear_scale_x")
            .get_parameter_value()
            .double_value
        )
        self.linear_scale_y = (
            self.get_parameter("linear_scale_y")
            .get_parameter_value()
            .double_value
        )
        self.pub_odom_tf = (
            self.get_parameter("pub_odom_tf").get_parameter_value().bool_value
        )

        # Create the TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create subscription to vel_raw
        self.subscription = self.create_subscription(
            Twist, "vel_raw", self.handle_vel, 50
        )

        # Create publisher for odom_raw
        self.odom_publisher = self.create_publisher(Odometry, "odom", 50)

        # Initialize variables
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def handle_vel(self, msg: Twist):
        current_time = self.get_clock().now()
        vel_dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Extract velocities
        linear_velocity_x = msg.linear.x * self.linear_scale_x
        linear_velocity_y = msg.linear.y * self.linear_scale_y

        # Calculate the steering angle and radius
        steer_angle = linear_velocity_y
        # steer_angle = msg.angular.z
        if abs(steer_angle) < 1e-6:
            R = float("inf")  # Effectively means going straight
            angular_velocity_z = 0.0
        else:
            R = self.wheelbase / tan(steer_angle / 180.0 * pi)
            angular_velocity_z = linear_velocity_x / R

        # Update the robot's position
        delta_theta = angular_velocity_z * vel_dt
        delta_x = linear_velocity_x * cos(self.theta) * vel_dt
        delta_y = linear_velocity_x * sin(self.theta) * vel_dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Prepare the odometry message
        odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3]
        )

        # Correct the covariance list to have exactly 36 elements, all as floats
        odom.pose.covariance = [
            float(0.001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.001),
        ]

        odom.twist.twist.linear.x = linear_velocity_x
        odom.twist.twist.linear.y = 0.0  # vy is set to 0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = angular_velocity_z
        odom.twist.covariance = [
            float(0.0001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.0001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.0001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.0001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.0001),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(0.0001),
        ]

        # Publish the odometry message
        self.odom_publisher.publish(odom)

        # Publish the TF transform if required
        if self.pub_odom_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_footprint_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = Quaternion(
                x=odom_quat[0], y=odom_quat[1], z=odom_quat[2], w=odom_quat[3]
            )
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
