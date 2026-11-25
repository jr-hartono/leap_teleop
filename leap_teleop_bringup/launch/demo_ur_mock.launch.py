"""Demo: drive a simulated UR5e (mock hardware, RViz) with the Leap controller.

Needs: sudo apt install ros-jazzy-moveit-servo ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config
"""

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEMO_PACKAGES = ["moveit_servo", "ur_robot_driver", "ur_moveit_config"]
UR_TYPE = "ur5e"

# Servo and MoveIt planning share UR's default controller, so no controller switch is needed.
CONTROLLER = "scaled_joint_trajectory_controller"
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


def servo_node(context):
    from moveit_configs_utils import MoveItConfigsBuilder

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": UR_TYPE})
        .to_moveit_configs()
    )
    servo_yaml = os.path.join(
        get_package_share_directory("ur_moveit_config"), "config", "ur_servo.yaml"
    )
    with open(servo_yaml) as file:
        servo_params = yaml.safe_load(file)
    servo_params.update(
        {
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "command_out_topic": f"/{CONTROLLER}/joint_trajectory",
            "publish_joint_velocities": True,
        }
    )
    return [
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[moveit_config.to_dict(), {"moveit_servo": servo_params}],
            output="screen",
        )
    ]


def robot(context):
    # Servo sends trajectories that end while the arm is still moving. UR's controller config
    # rejects those, so the demo uses a copy that allows them.
    controllers = os.path.join(
        get_package_share_directory("ur_robot_driver"), "config", "ur_controllers.yaml"
    )
    with open(controllers) as file:
        text = file.read()
    section = f"{CONTROLLER}:\n  ros__parameters:\n"
    if section not in text:
        raise RuntimeError(f"{CONTROLLER} not found in {controllers}")
    text = text.replace(
        section, section + "    allow_nonzero_velocity_at_trajectory_end: true\n", 1
    )
    with tempfile.NamedTemporaryFile(
        "w", prefix="demo_ur_controllers_", suffix=".yaml", delete=False
    ) as patched:
        patched.write(text)
    return [
        include(
            "ur_robot_driver",
            "ur_control.launch.py",
            {
                "ur_type": UR_TYPE,
                "robot_ip": "0.0.0.0",
                "use_mock_hardware": "true",
                "initial_joint_controller": CONTROLLER,
                "controllers_file": patched.name,
                "launch_rviz": "false",
                "launch_dashboard_client": "false",
            },
        )
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
            OpaqueFunction(function=check_demo_packages),
            OpaqueFunction(function=robot),
            include(
                "ur_moveit_config",
                "ur_moveit.launch.py",
                {
                    "ur_type": UR_TYPE,
                    "launch_servo": "false",
                    "launch_rviz": "true",
                },
            ),
            OpaqueFunction(function=servo_node),
            switch_to_twist,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=switch_to_twist, on_exit=[move_to_start_pose]
                )
            ),
            include("leap_teleop_bringup", "teleop.launch.py", {"dry_run": "false"}),
        ]
    )
