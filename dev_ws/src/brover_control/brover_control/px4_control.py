#!/usr/bin/env python3
import rclpy
from px4_msgs.msg import (
    ManualControlSetpoint,
    OffboardControlMode,
    VehicleCommand,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Joy


class PX4Controller(Node):
    def __init__(self):
        super().__init__("px4_controller_node")

        self.declare_parameter("arm_button", 7)
        self.declare_parameter("disarm_button", 6)
        self.declare_parameter("offboard_button", 0)
        self.declare_parameter("manual_button", 1)

        self.arm_button = (
            self.get_parameter("arm_button")
            .get_parameter_value()
            .integer_value
        )
        self.disarm_button = (
            self.get_parameter("disarm_button")
            .get_parameter_value()
            .integer_value
        )
        self.offboard_button = (
            self.get_parameter("offboard_button")
            .get_parameter_value()
            .integer_value
        )
        self.manual_button = (
            self.get_parameter("manual_button")
            .get_parameter_value()
            .integer_value
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )

        self.manual_control_publisher = self.create_publisher(
            ManualControlSetpoint,
            "/fmu/in/manual_control_setpoint",
            qos_profile,
        )

        self.joy_subscriber = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        self.send_command_timer = self.create_timer(0.1, self.send_command)
        self.cmd_loop_timer = self.create_timer(0.2, self.cmdloop_callback)

        self.current_mode = None
        self.joystick_state = None

        self.roll = self.pitch = self.yaw = self.throttle = 0.0

    def joy_callback(self, msg):
        self.set_control(
            roll=-msg.axes[3],
            pitch=msg.axes[0],
            yaw=msg.axes[4],
            throttle=(msg.axes[1]),
        )

        self.joystick_state = msg.buttons

        if self.joystick_state[self.arm_button]:
            self.current_mode = "ARM"
        elif self.joystick_state[self.disarm_button]:
            self.current_mode = "DISARM"
        elif self.joystick_state[self.offboard_button]:
            self.current_mode = "OFFBOARD"
        elif self.joystick_state[self.manual_button]:
            self.current_mode = "MANUAL"

    def set_control(self, roll, pitch, yaw, throttle):
        self.roll = max(-1.0, min(roll, 1.0))
        self.pitch = max(-1.0, min(pitch, 1.0))
        self.yaw = max(-1.0, min(yaw, 1.0))
        self.throttle = max(-1.0, min(throttle, 1.0))

        self.send_manual_control()

    def send_manual_control(self):
        msg_mc = ManualControlSetpoint()
        msg_mc.timestamp = self.get_clock().now().nanoseconds // 1000

        msg_mc.roll = self.roll
        msg_mc.pitch = self.pitch
        msg_mc.yaw = self.yaw
        msg_mc.throttle = self.throttle
        msg_mc.yaw = self.yaw
        msg_mc.valid = True

        self.manual_control_publisher.publish(msg_mc)

    def send_command(self):
        if self.current_mode is None:
            return

        if self.current_mode == "ARM":
            command = 400
            param1 = 1.0  # arm
            self.publish_vehicle_command(command, param1)
            # self.get_logger().info("Arming the vehicle ...")
        elif self.current_mode == "DISARM":
            command = 400
            param1 = 0.0  # disarm
            self.publish_vehicle_command(command, param1)
            # self.get_logger().info("Disarming the vehicle ...")
        elif self.current_mode == "OFFBOARD":
            command = 176
            param1 = 1.0  # custom mode flag for offboard
            param2 = 6.0  # offboard
            self.publish_vehicle_command(command, param1, param2)
            # self.get_logger().info("Switching to OFFBOARD mode ...")
        elif self.current_mode == "MANUAL":
            command = 176
            param1 = 1.0  # custom mode flag for manual
            param2 = 1.0  # manual
            self.publish_vehicle_command(command, param1, param2)
            # self.get_logger().info("Switching to MANUAL mode ...")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(msg)

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.attitude = True
        offboard_msg.acceleration = False

        self.offboard_mode_publisher.publish(offboard_msg)


def main(args=None):
    rclpy.init(args=args)
    px4_controller = PX4Controller()
    rclpy.spin(px4_controller)
    px4_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
