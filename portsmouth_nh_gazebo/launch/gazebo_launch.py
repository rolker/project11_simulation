"""Launch file for Portsmouth NH Gazebo world.

Generates the world from chart/bathymetry data (if not already cached),
then launches Gazebo Harmonic with the generated SDF.
"""

import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


def _generate_world(context, *args, **kwargs):
    """Generate the world SDF if it doesn't already exist."""
    verbose = LaunchConfiguration('verbose').perform(context) == 'true'
    pkg_share = get_package_share_directory('portsmouth_nh_gazebo')
    config_path = os.path.join(pkg_share, 'config', 'portsmouth.yaml')

    with open(config_path) as f:
        config = yaml.safe_load(f)

    bounds = config['bounds']
    world_name = config['world_name']
    grid_power = config.get('grid_power', 9)

    # Output directory for generated world
    cache_base = os.environ.get(
        'XDG_CACHE_HOME', os.path.expanduser('~/.cache')
    )
    output_dir = os.path.join(cache_base, 'portsmouth_nh_gazebo', world_name)

    sdf_path = os.path.join(output_dir, f'{world_name}.sdf')

    # Only regenerate if the SDF doesn't exist
    if not os.path.exists(sdf_path):
        cmd = [
            sys.executable, '-m',
            'marine_charts_to_gazebo_world.generate_world',
            '--bounds',
            f"{bounds['south']},{bounds['west']},"
            f"{bounds['north']},{bounds['east']}",
            '--output-dir', output_dir,
            '--world-name', world_name,
            '--grid-power', str(grid_power),
        ]

        enc_root = config.get('enc_root') or os.environ.get(
            'ROS_S57_ENC_ROOT'
        )
        if enc_root:
            cmd.extend(['--enc-root', enc_root])

        print(f'[portsmouth_nh_gazebo] Generating world: {world_name}')
        subprocess.check_call(cmd)
        print(f'[portsmouth_nh_gazebo] World generated: {sdf_path}')
    else:
        print(f'[portsmouth_nh_gazebo] Using cached world: {sdf_path}')

    # Return actions to set resource path and launch gz sim
    return [
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=output_dir + os.pathsep + os.environ.get(
                'GZ_SIM_RESOURCE_PATH', ''
            ),
        ),
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
        OpaqueFunction(function=_generate_world),
    ])
