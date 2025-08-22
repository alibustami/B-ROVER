from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    vehicle_startup = LaunchConfiguration("vehicle_startup")
    declare_args = [
        DeclareLaunchArgument("vehicle_startup", default_value="true"),
    ]

    robot_urdf = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("brover_description"),
                        "urdf",
                        "brover.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    robot_description = {"robot_description": robot_urdf}

    vehicle_group = GroupAction(
        condition=IfCondition(vehicle_startup),
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    robot_description,
                    {"publish_description": True, "use_sim_time": False},
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="both",
            ),
            Node(package="joy", executable="joy_node", name="joy_node_ws"),
            Node(
                package="brover_control", executable="bridge.py", name="bridge"
            ),
            Node(
                package="brover_control",
                executable="odom_pub.py",
                name="odom_pub",
            ),
            Node(
                package="brover_control",
                executable="px4_control.py",
                name="px4_control",
            ),
        ],
    )

    return LaunchDescription(declare_args + [vehicle_group])
