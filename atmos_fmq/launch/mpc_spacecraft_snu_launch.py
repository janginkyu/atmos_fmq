from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='wrench',
        description='Mode of the controller (rate, wrench, direct_allocation)'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    mode = LaunchConfiguration('mode')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        mode_arg,
        namespace_arg,
        Node(
            package='px4_mpc',
            namespace=namespace,
            executable='mpc_spacecraft',
            name='mpc_spacecraft',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'mode': mode},
                {'namespace': namespace},
            ]
        ),
    ])
