"""Demo: drive a simulated UR5e (mock hardware, RViz) with the Leap controller.

Needs: sudo apt install ros-jazzy-moveit-servo ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config
"""

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
from launch_ros.substitutions import FindPackageShare

DEMO_PACKAGES = ["moveit_servo", "ur_robot_driver", "ur_moveit_config"]

# Elbow bent, tool pointing down: away from the singularities of UR's all-zero default.
START_POSE = "[0.0, -1.57, 1.57, -1.57, -1.57, 0.0]"


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


def include(package, launch_file, arguments):
    return IncludeLaunchDescription(
        [FindPackageShare(package), "/launch/", launch_file],
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
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
            "topic",
            "pub",
            "-t",
            "20",
            "-r",
            "10",
            "/forward_position_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            f"{{data: {START_POSE}}}",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=check_demo_packages),
            include(
                "ur_robot_driver",
                "ur_control.launch.py",
                {
                    "ur_type": ur_type,
                    "robot_ip": "0.0.0.0",
                    "use_mock_hardware": "true",
                    "initial_joint_controller": "forward_position_controller",
                    "launch_rviz": "false",
                    "launch_dashboard_client": "false",
                },
            ),
            include(
                "ur_moveit_config",
                "ur_moveit.launch.py",
                {
                    "ur_type": ur_type,
                    "launch_servo": "true",
                    "launch_rviz": LaunchConfiguration("rviz"),
                },
            ),
            switch_to_twist,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=switch_to_twist, on_exit=[move_to_start_pose]
                )
            ),
            include("leap_teleop_bringup", "teleop.launch.py", {"dry_run": "false"}),
        ]
    )
