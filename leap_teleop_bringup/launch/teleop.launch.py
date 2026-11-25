import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def check_tracker_built():
    tracker = os.path.join(
        get_package_prefix("leap_teleop_core"),
        "lib",
        "leap_teleop_core",
        "leap_tracker",
    )
    if not os.path.exists(tracker):
        raise RuntimeError(
            "leap_tracker was not built because LeapSDK was not found. Install the Ultraleap "
            "Hand Tracking Service, then rebuild: colcon build --cmake-clean-cache"
        )


def launch_setup(context, *args):
    check_tracker_built()
    namespace = LaunchConfiguration("namespace").perform(context)
    if namespace == "/" or not namespace:
        leap_namespace = "/leap_teleop"
    else:
        leap_namespace = namespace.rstrip("/") + "/leap_teleop"
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"

    parameters = [params_file, {"use_sim_time": use_sim_time}]
    bridge_parameters = parameters + [{"dry_run": dry_run}]

    nodes = [
        Node(
            package="leap_teleop_core",
            executable="leap_tracker",
            name="leap_tracker",
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package="leap_teleop_core",
            executable="leap_teleop_deadman",
            name="leap_teleop_deadman",
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package="leap_teleop_core",
            executable="leap_teleop_supervisor",
            name="leap_teleop_supervisor",
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package="leap_teleop_bridge",
            executable="leap_moveit_servo_bridge",
            name="leap_moveit_servo_bridge",
            namespace=leap_namespace,
            parameters=bridge_parameters,
        ),
    ]
    return [LaunchDescription(nodes)]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("leap_teleop_bringup"), "config", "example_arm.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="/",
                description="Outer namespace; the stack adds /leap_teleop.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameter file with one section per node.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Log bridge commands without publishing them.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
