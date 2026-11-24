from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace == '/' or not namespace:
        leap_namespace = '/leap_teleop'
    else:
        leap_namespace = namespace.rstrip('/') + '/leap_teleop'
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    parameters = [params_file, {'use_sim_time': use_sim_time}]

    nodes = [
        Node(
            package='leap_teleop_core',
            executable='leap_tracker',
            name='leap_tracker',
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package='leap_teleop_core',
            executable='leap_teleop_engage',
            name='leap_teleop_engage',
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package='leap_teleop_core',
            executable='leap_teleop_supervisor',
            name='leap_teleop_supervisor',
            namespace=leap_namespace,
            parameters=parameters,
        ),
        Node(
            package='leap_teleop_bridge',
            executable='leap_moveit_servo_bridge',
            name='leap_moveit_servo_bridge',
            namespace=leap_namespace,
            parameters=parameters,
        ),
    ]
    return [LaunchDescription(nodes)]


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare('leap_teleop_bringup'), 'config', 'example_arm.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/',
                              description='Outer namespace; the stack adds /leap_teleop.'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Parameter file with one section per node.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time.'),
        OpaqueFunction(function=launch_setup),
    ])
