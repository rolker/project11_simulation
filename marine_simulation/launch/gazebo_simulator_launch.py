# Copyright 2026 Roland Arsenault, UNH CCOM
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch full Gazebo simulation: Portsmouth Harbor + BEN + autonomy stack.

Parallel to simulator_launch.py but uses Gazebo instead of asv_sim for
physics and gazebo_helm instead of asv_helm for thruster control.

Includes gazebo_ben_launch.py for the Gazebo world and BEN model, then
adds the autonomy stack, gazebo helm, and operator UI on top.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition


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

    # 1–2. Gazebo world + BEN model
    gazebo_ben = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('marine_simulation'),
                'launch',
                'gazebo_ben_launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
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
            'use_sim_time': 'true',
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
        gazebo_ben,
        ben_core,
        gazebo_helm_group,
        sim_operator,
    ])
