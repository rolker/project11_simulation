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

"""Launch file for Portsmouth NH Gazebo world.

The world SDF is generated at build time by CMake. This launch file
simply locates the installed SDF and starts Gazebo Harmonic.
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


def _launch_gazebo(context, *args, **kwargs):
    """Locate the pre-built world SDF and launch Gazebo."""
    verbose = LaunchConfiguration('verbose').perform(context) == 'true'

    pkg_share = get_package_share_directory('portsmouth_nh_gazebo')
    world_name = 'portsmouth_harbor'
    sdf_path = os.path.join(
        pkg_share, 'worlds', world_name, f'{world_name}.sdf'
    )

    if not os.path.exists(sdf_path):
        raise RuntimeError(
            f'World SDF not found at {sdf_path}. '
            'Rebuild the package: colcon build --packages-select '
            'portsmouth_nh_gazebo'
        )

    return [
        ExecuteProcess(
            cmd=['gz', 'sim', '-v4' if verbose else '-v1', sdf_path],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose Gazebo output',
        ),
        OpaqueFunction(function=_launch_gazebo),
    ])
