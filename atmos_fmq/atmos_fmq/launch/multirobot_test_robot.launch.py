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

    namespaces = LaunchConfiguration('namespaces')
    namespaces_list = PythonExpression([
        "str('", namespaces, "').split(',')"
    ])

    return LaunchDescription([
        namespaces_arg,

        Node(
            package='atmos_fmq',
            namespace='',
            executable='robot',
            name='control_feeder',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'namespaces': namespaces_list},
            ],
        ),
    ])
