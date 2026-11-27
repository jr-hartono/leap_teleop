"""Demo: drive a simulated UR5e with a Robotiq 2F-85 (mock hardware) with the Leap controller.

Needs: the packages in DEMO_PACKAGES; the README lists the apt command.
"""

import os

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEMO_PACKAGES = [
    "moveit_servo",
    "ur_description",
    "ur_moveit_config",
    "robotiq_description",
    "controller_manager",
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "forward_command_controller",
    "robot_state_publisher",
    "xacro",
]

CONTROLLER = "joint_trajectory_controller"
JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Elbow bent, tool pointing down: away from the singularities of UR's default pose.
START_POSE = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


def config_path(name):
    return os.path.join(
        get_package_share_directory("leap_teleop_bringup"),
        "config",
        "demo_ur5e_2f85",
        name,
    )


def load_yaml(path):
    with open(path) as file:
        return yaml.safe_load(file)


def check_demo_packages(context):
    missing = []
    for package in DEMO_PACKAGES:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            missing.append(package)
    if missing:
        apt = " ".join("ros-jazzy-" + p.replace("_", "-") for p in missing)
        raise RuntimeError(
            f"The demo needs {', '.join(missing)}. Install with:\n"
            f"  sudo apt install {apt}"
        )
    return []


def robot(context):
    import xacro

    robot_description = xacro.process_file(config_path("robot.urdf.xacro")).toxml()
    robot_description_semantic = xacro.process_file(
        config_path("robot.srdf.xacro")
    ).toxml()
    # Arm kinematics and acceleration limits come with the UR MoveIt config.
    ur_moveit_config = os.path.join(
        get_package_share_directory("ur_moveit_config"), "config"
    )
    servo_parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": load_yaml(
            os.path.join(ur_moveit_config, "kinematics.yaml")
        ),
        "robot_description_planning": load_yaml(
            os.path.join(ur_moveit_config, "joint_limits.yaml")
        ),
        "moveit_servo": load_yaml(config_path("servo.yaml")),
    }
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[config_path("controllers.yaml")],
            remappings=[("~/robot_description", "/robot_description")],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                CONTROLLER,
                "gripper_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[servo_parameters],
            output="screen",
        ),
    ]


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        [FindPackageShare(package), "/launch/", launch_file],
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    # Servo ignores command_in_type from YAML and must be switched to TWIST (1) at runtime.
    # The call blocks until Servo is up.
    switch_to_twist = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/servo_node/switch_command_type",
            "moveit_msgs/srv/ServoCommandType",
            "{command_type: 1}",
        ]
    )
    # Servo waits until the joint state changes, which a still mock arm never does on its own.
    move_to_start_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "action",
            "send_goal",
            f"/{CONTROLLER}/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            yaml.safe_dump(
                {
                    "trajectory": {
                        "joint_names": JOINTS,
                        "points": [
                            {"positions": START_POSE, "time_from_start": {"sec": 2}}
                        ],
                    }
                },
                default_flow_style=True,
            ),
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=config_path("leap_teleop.yaml"),
                description="leap_teleop parameter file. A copy should keep the demo's "
                "tool frames and gripper range.",
            ),
            OpaqueFunction(function=check_demo_packages),
            OpaqueFunction(function=robot),
            switch_to_twist,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=switch_to_twist, on_exit=[move_to_start_pose]
                )
            ),
            include(
                "leap_teleop_bringup",
                "teleop.launch.py",
                {
                    "dry_run": "false",
                    "gripper": "true",
                    "params_file": LaunchConfiguration("params_file"),
                },
            ),
        ]
    )
