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

"""Shared launch helper for portsmouth_nh_gazebo worlds.

Each world launch file calls make_gazebo_launch() with its world name.
The world SDF is generated at build time by CMake.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration

PACKAGE_NAME = 'portsmouth_nh_gazebo'


def make_gazebo_launch(world_name: str) -> LaunchDescription:
    """Create a LaunchDescription that starts Gazebo with the named world."""

    def _launch_gazebo(context, *args, **kwargs):
        verbose = LaunchConfiguration('verbose').perform(context) == 'true'

        pkg_share = get_package_share_directory(PACKAGE_NAME)
        sdf_path = os.path.join(
            pkg_share, 'worlds', world_name, f'{world_name}.sdf'
        )

        if not os.path.exists(sdf_path):
            raise RuntimeError(
                f'World SDF not found at {sdf_path}. '
                f'Rebuild the package: colcon build --packages-select '
                f'{PACKAGE_NAME}'
            )

        return [
            ExecuteProcess(
                cmd=['gz', 'sim', '-v4' if verbose else '-v1', sdf_path],
                output='screen',
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose Gazebo output',
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
