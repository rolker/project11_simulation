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

"""Launch Gazebo Portsmouth Harbor world and spawn BEN.

Starts the Gazebo simulator with the Portsmouth NH Harbor world and
spawns the BEN model into it. Can be used standalone for visualization
or sensor testing, or included from a full simulation launch file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


WORLD_NAME = 'portsmouth_nh_harbor'


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='ben'))

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
            'x': '-200',
            'y': '-556',
        }.items(),
    )

    return LaunchDescription([
        namespace_arg,
        set_use_sim_time,
        gz_harbor,
        spawn_ben,
    ])
