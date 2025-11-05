from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParametersFromFile
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        SetParametersFromFile(
            PathJoinSubstitution([
                FindPackageShare('project11_simulation'),
                'config',
                'udp_bridge.yaml'
            ])
        ),
        GroupAction(
            actions=[
                PushRosNamespace('ben'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('udp_bridge'),
                            'launch',
                            'udp_bridge_launch.py'
                        ])
                    )
                ),
            ]
        ),

        GroupAction(
            actions=[
                PushRosNamespace('operator'),
                SetEnvironmentVariable(
                    name='ROS_DOMAIN_ID',
                    value='1'
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('udp_bridge'),
                            'launch',
                            'udp_bridge_launch.py'
                        ])
                    )
                ),
            ]
        ),
    ])
