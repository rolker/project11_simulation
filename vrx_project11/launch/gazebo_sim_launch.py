from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf',
            default_value=PathJoinSubstitution([
                FindPackageShare('vrx_project11'),
                'urdf', 'wamv.urdf'
            ]),
            description='URDF file to use for the robot'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('vrx_gz'),
                    'launch',
                    'competition.launch.py'
                ])
            ),
            launch_arguments={
                'urdf': LaunchConfiguration('urdf'),
            }.items()
        ),
    ])