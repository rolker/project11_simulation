"""Launch full Gazebo simulation: Portsmouth Harbor + BEN + autonomy stack.

Parallel to simulator_launch.py but uses Gazebo instead of asv_sim for
physics and gazebo_helm instead of asv_helm for thruster control.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition


WORLD_NAME = 'portsmouth_nh_harbor'


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    background_chart = LaunchConfiguration('background_chart')
    enable_bridge = LaunchConfiguration('enable_bridge')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='ben'))
    background_chart_arg = DeclareLaunchArgument(
        'background_chart', default_value=PathJoinSubstitution(
            [FindPackageShare('camp'), 'workspace', '13283', '13283_2.KAP']
        ))
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge', default_value=TextSubstitution(text='false'))

    # Global use_sim_time for all nodes
    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

    # 1. Start Gazebo with Portsmouth Harbor world
    gz_harbor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('portsmouth_nh_gazebo'),
                'launch',
                'harbor_launch.py'
            ])
        ),
    )

    # 2. Spawn BEN into the Portsmouth Harbor world
    spawn_ben = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ben_gazebo'),
                'launch',
                'spawn_ben_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'world_name': WORLD_NAME,
        }.items(),
    )

    # 3. Autonomy stack (skip robot_state_publisher — Gazebo provides one)
    ben_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ben_project11'),
                'launch',
                'ben_core_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'enable_bridge': enable_bridge,
            'is_simulator': 'true',
            'launch_robot_state_publisher': 'false',
        }.items(),
    )

    # 4. Gazebo helm (replaces asv_helm for Gazebo simulation)
    #    Remappings match the sim_robot_launch.py pattern
    gazebo_helm_group = GroupAction(
        actions=[
            PushROSNamespace(namespace),
            SetRemap(src='helm', dst='marine/control/helm'),
            SetRemap(src='cmd_vel', dst='marine/control/cmd_vel'),
            SetRemap(src='odom', dst='odom'),
            LifecycleNode(
                package='ben_gazebo',
                executable='gazebo_helm',
                name='gazebo_helm',
                namespace='',
                respawn=True,
                respawn_delay=2,
                emulate_tty=True,
            ),
            LifecycleTransition(
                lifecycle_node_names=(
                    PythonExpression(
                        expression=[
                            '"',
                            namespace,
                            '" + "/gazebo_helm"'
                        ],
                    ),
                ),
                transition_ids=(
                    Transition.TRANSITION_CONFIGURE,
                    Transition.TRANSITION_ACTIVATE,
                ),
            ),
        ],
    )

    # 5. Operator UI + RViz
    sim_operator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('marine_simulation'),
                'launch',
                'sim_operator_launch.py'
            ])
        ),
        launch_arguments={
            'robot_namespace': namespace,
            'operator_namespace': 'operator',
            'enable_bridge': enable_bridge,
            'background_chart': background_chart,
            'rviz': 'true',
            'rviz_configuration': PathJoinSubstitution([
                FindPackageShare('ben_project11'),
                'config',
                'ben.rviz'
            ]),
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        background_chart_arg,
        enable_bridge_arg,
        set_use_sim_time,
        gz_harbor,
        spawn_ben,
        ben_core,
        gazebo_helm_group,
        sim_operator,
    ])
