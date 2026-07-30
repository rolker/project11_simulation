from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value=TextSubstitution(text="operator")
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=TextSubstitution(text="wamv")
        ),
        SetParameter(
            name='use_sim_time',
            value=True
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('marine_autonomy'), '/launch/operator_core_launch.py']),
            launch_arguments={
                'robot_namespace': robot_namespace,
                'enable_bridge': 'false'
            }.items()
        ),
        GroupAction(
            actions=[
                PushROSNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('marine_autonomy'),
                            'launch',
                            'operator_ui_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'namespace': namespace,
                        'robot_namespace': robot_namespace,
                    }.items()
                ),
            ]
        )
    ])
