# Copyright (c) 2026 Agile Robots
#
# Diana7 Bringup based on franka.launch.py
#
# This launch file intentionally duplicates the structure of
# franka.launch.py to minimize changes to upstream files and
# simplify future merges from the official franka_ros2 repository.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_robot_nodes(context):

    load_gripper_launch_configuration = LaunchConfiguration("load_gripper").perform(
        context
    )

    load_gripper = load_gripper_launch_configuration.lower() == "true"

    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)

    namespace = LaunchConfiguration("namespace").perform(context)

    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)

    joint_state_rate = int(LaunchConfiguration("joint_state_rate").perform(context))

    #
    # Diana Description
    #
    urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("diana_description"),
            "robots",
            "diana7",
            "diana7.urdf.xacro",
        ]
    ).perform(context)

    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "arm_prefix": arm_prefix,
            "hand": load_gripper_launch_configuration,
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(
                context
            ),
            "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands").perform(
                context
            ),
        },
    ).toprettyxml(indent="  ")

    joint_state_publisher_sources = [
        "franka/joint_states",
        "franka_gripper/joint_states",
    ]

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {
                    "robot_type": "diana7",
                },
                {
                    "load_gripper": load_gripper,
                },
                {
                    "arm_prefix": arm_prefix,
                },
            ],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            namespace=namespace,
            parameters=[
                {
                    "source_list": joint_state_publisher_sources,
                    "rate": joint_state_rate,
                }
            ],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=[
                "joint_state_broadcaster",
                "--controller-ros-args",
                f"--remap joint_states:={joint_state_publisher_sources[0]}",
            ],
            output="screen",
        ),
    ]

    load_broadcaster = (
        LaunchConfiguration("load_franka_robot_state_broadcaster")
        .perform(context)
        .lower()
        == "true"
    )

    use_fake_hardware = (
        LaunchConfiguration("use_fake_hardware").perform(context).lower() == "true"
    )

    if load_broadcaster and not use_fake_hardware:

        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=["franka_robot_state_broadcaster"],
                output="screen",
            )
        )

    # nodes.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("franka_gripper"),
    #                     "launch",
    #                     "gripper.launch.py",
    #                 ]
    #             )
    #         ),
    #         launch_arguments={
    #             "namespace": namespace,
    #             "robot_ip": LaunchConfiguration("robot_ip").perform(context),
    #             "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(
    #                 context
    #             ),
    #         }.items(),
    #         condition=IfCondition(LaunchConfiguration("load_gripper")),
    #     )
    # )

    return nodes


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
        ),
        DeclareLaunchArgument(
            "robot_type",
            default_value="diana7",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.10.75",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
        ),
        DeclareLaunchArgument(
            "joint_state_rate",
            default_value="30",
        ),
        DeclareLaunchArgument(
            "load_franka_robot_state_broadcaster",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("franka_bringup"),
                    "config",
                    "diana7.ros2_controllers.yaml",
                ]
            ),
        ),
    ]

    return LaunchDescription(
        launch_args + [OpaqueFunction(function=generate_robot_nodes)]
    )
