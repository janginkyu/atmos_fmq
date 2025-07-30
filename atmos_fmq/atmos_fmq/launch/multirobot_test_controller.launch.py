from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespaces_arg = DeclareLaunchArgument(
        'namespaces',
        description='robot names as a list of strings'
    )
    simulated_delay_arg = DeclareLaunchArgument(
        'simulated_delay',
        description='use simulated delay',
        default_value='False'
    )

    namespaces = LaunchConfiguration('namespaces')
    namespaces_list = PythonExpression([
        "str('", namespaces, "').split(',')"
    ])

    simulated_delay_ = LaunchConfiguration('simulated_delay')
    simulated_delay = PythonExpression([
        "bool(", simulated_delay_, ")"
    ])

    return LaunchDescription([
        namespaces_arg,
        simulated_delay_arg,

        Node(
            package='atmos_fmq',
            namespace='',
            executable='wrench_control',
            name='remote_control',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'namespaces': namespaces_list,
                    'simulated_delay': simulated_delay
                },
            ],
        ),
        Node(
            package='atmos_fmq',
            namespace='',
            executable='publish_setpoints',
            name='setpoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'namespaces': namespaces_list},
            ],
        ),
        Node(
            package='atmos_fmq',
            namespace='',
            executable='delay_simulator',
            name='delay_simulator',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(simulated_delay)
        )

    ])
