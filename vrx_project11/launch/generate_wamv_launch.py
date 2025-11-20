from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wamv_target', description='Path to output WAM-V urdf file.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('vrx_gazebo'),
                    'launch',
                    'generate_wamv.launch.py'
                ])
            ),
            launch_arguments={
                'component_yaml': PathJoinSubstitution([
                    FindPackageShare('vrx_project11'),
                    'config', 'wamv_config', 'component_config.yaml'
                ]),
                'wamv_target': LaunchConfiguration('wamv_target'),
                'alternative_macros_file': PathJoinSubstitution([
                    FindPackageShare('vrx_project11'),
                    'urdf', 'macros.xacro'
                ]),
            }.items()
        ),
    ])

