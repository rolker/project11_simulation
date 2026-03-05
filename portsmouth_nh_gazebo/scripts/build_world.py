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

"""Build-time wrapper for world generation.

Reads the YAML config and invokes generate_world with the appropriate
arguments. Called by CMake during the build phase.
"""

import argparse
import os
import sys

import yaml

from marine_charts_to_gazebo_world.generate_world import main as generate_main


def main():
    parser = argparse.ArgumentParser(
        description='Build-time world generation from config file.'
    )
    parser.add_argument(
        '--config', required=True,
        help='Path to YAML config file (e.g. portsmouth.yaml)',
    )
    parser.add_argument(
        '--output-dir', required=True,
        help='Directory to write generated world files into.',
    )
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    bounds = config['bounds']
    world_name = config['world_name']
    grid_power = config.get('grid_power', 9)

    gen_args = [
        '--bounds',
        f"{bounds['south']},{bounds['west']},{bounds['north']},{bounds['east']}",
        '--output-dir', args.output_dir,
        '--world-name', world_name,
        '--grid-power', str(grid_power),
    ]

    # ENC root: config override takes precedence, then env var
    enc_root = config.get('enc_root') or os.environ.get('ROS_S57_ENC_ROOT')
    if enc_root:
        if os.path.isdir(enc_root):
            gen_args.extend(['--enc-root', enc_root])
        else:
            print(
                f'Warning: ENC root {enc_root!r} not found, '
                'skipping S57 chart data',
                file=sys.stderr,
            )
    else:
        print(
            'Warning: No ENC root configured '
            '(set ROS_S57_ENC_ROOT or enc_root in config), '
            'skipping S57 chart data',
            file=sys.stderr,
        )

    sys.exit(generate_main(gen_args))


if __name__ == '__main__':
    main()
