import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def check_driver_built():
    driver = os.path.join(
        get_package_prefix("leap_teleop_core"),
        "lib",
        "leap_teleop_core",
        "leap_driver",
    )
    if not os.path.exists(driver):
        raise RuntimeError(
            "leap_driver was not built because LeapSDK was not found. Install the Ultraleap "
            "Hand Tracking Service, then rebuild: colcon build --cmake-clean-cache"
        )


def launch_setup(context, *args):
    check_driver_built()
    namespace = LaunchConfiguration("namespace").perform(context)
    if namespace == "/" or not namespace:
        leap_namespace = "/leap_teleop"
    else:
        leap_namespace = namespace.rstrip("/") + "/leap_teleop"
    params_file = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
    side = LaunchConfiguration("side").perform(context)
    mount_mode = LaunchConfiguration("mount_mode").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    parameters = [params_file, {"use_sim_time": use_sim_time}]
    side_parameters = parameters + [{"side": side}]
    # The driver requests the service's tracking mode, the mapper maps the device axes.
    driver_parameters = parameters + [{"mount_mode": mount_mode}]
    mapper_parameters = side_parameters + [{"mount_mode": mount_mode}]
    bridge_parameters = side_parameters + [{"dry_run": dry_run}]

    nodes = [
        Node(
            package="leap_teleop_core",
            executable="leap_driver",
            name="leap_driver",
            namespace=leap_namespace,
            parameters=driver_parameters,
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
            executable="leap_teleop_mapper",
            name="leap_teleop_mapper",
            namespace=leap_namespace,
            parameters=mapper_parameters,
        ),
        Node(
            package="leap_teleop_bridge",
            executable="leap_moveit_servo_bridge",
            name="leap_moveit_servo_bridge",
            namespace=leap_namespace,
            parameters=bridge_parameters,
        ),
    ]
    if rviz:
        nodes += [
            Node(
                package="leap_teleop_viz",
                executable="leap_teleop_viz",
                name="leap_teleop_viz",
                namespace=leap_namespace,
                parameters=side_parameters,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="leap_teleop_rviz",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("leap_teleop_viz"),
                        "rviz",
                        "leap_teleop.rviz",
                    ),
                ],
                # The config subscribes to /leap_teleop/markers.
                remappings=[("/leap_teleop/markers", leap_namespace + "/markers")],
                parameters=[{"use_sim_time": use_sim_time}],
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
                "side",
                default_value="right",
                description="Hand that teleoperates the arm: right | left.",
            ),
            DeclareLaunchArgument(
                "mount_mode",
                default_value="desktop",
                description="Leap device mounting, for the driver and the mapper: "
                "desktop | head_mounted.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="Log bridge commands without publishing them.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start leap_teleop_viz and an RViz window showing its markers.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
