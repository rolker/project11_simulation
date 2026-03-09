# Copyright 2025 Roland Arsenault
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Gazebo with the Portsmouth to Isles of Shoals world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration

WORLD_NAME = 'portsmouth_nh_portsmouth_to_isles'


def _launch_gazebo(context, *args, **kwargs):
    verbose = LaunchConfiguration('verbose').perform(context) == 'true'
    pkg_share = get_package_share_directory('portsmouth_nh_gazebo')
    sdf_path = os.path.join(
        pkg_share, 'worlds', WORLD_NAME, f'{WORLD_NAME}.sdf'
    )
    if not os.path.exists(sdf_path):
        raise RuntimeError(
            f'World SDF not found at {sdf_path}. '
            'Rebuild: colcon build --packages-select portsmouth_nh_gazebo'
        )
    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4' if verbose else '-v1', sdf_path],
            output='screen',
            on_exit=Shutdown(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Enable verbose Gazebo output',
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
