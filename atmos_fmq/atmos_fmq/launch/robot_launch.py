from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='wrench',
        description='Mode of the controller (rate, wrench, direct_allocation)'
    )

    # Namespace: Mandatory argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace for all nodes'
    )

    # Docker Path: path to the fleetmq docker file file
    sub_launch_path = os.path.join(
        get_package_share_directory('ros_fmq_bridge'),
        'launch',
        'fleetmq.launch.py'
    )

    # TODO(@Pedro-Roque): I removed the default docker-compose path, we can add it later
    docker_compose_path_arg = DeclareLaunchArgument(
        'docker_compose_path',
        # sdefault_value=os.path.join(get_package_share_directory('ros_fmq_bridge'), '/docker/docker-compose-vehicle-linux.yml'),
        description='Path to the Docker Compose file for the vehicle'
    )

    mode    = LaunchConfiguration('mode')
    namespace = LaunchConfiguration('namespace')
    docker_compose_path = LaunchConfiguration('docker_compose_path')

    # Clean namespace string to remove leading/trailing slashes
    clean_namespace = PythonExpression([
        "str('", namespace, "').strip('/')"
    ])

    filename = [
        docker_compose_path,
        TextSubstitution(text='/docker-compose-vehicle-linux_'),
        clean_namespace,
        TextSubstitution(text='.yml'),
    ]

    return LaunchDescription([
        mode_arg,
        namespace_arg,
        docker_compose_path_arg,
        Node(
            package='px4_mpc',
            namespace=namespace,
            executable='mpc_spacecraft',
            name='mpc_spacecraft',
            output='log',
            emulate_tty=True,
            parameters=[
                {'mode': mode},
                {'namespace': namespace},
            ]
        ),

        Node(
            package='atmos_fmq',
            namespace=namespace,
            executable='robot',
            name='robot_fmq',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'namespace': namespace},
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sub_launch_path)
        ),

        # Launch docker: docker compose -f docker-compose-vehicle-linux_{namespace}.yml up
        ExecuteProcess(
            cmd=['docker', 'compose', '-f', filename, 'up'],
            output='screen',
            cwd=get_package_share_directory('atmos_fmq')
        ),

    ])
