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


def _validate_config(config, config_path):
    """Validate required config keys and return clear errors."""
    if not isinstance(config, dict):
        print(
            f'Error: {config_path} must contain a YAML mapping.',
            file=sys.stderr,
        )
        return False

    required = ('bounds', 'world_name')
    missing = [k for k in required if k not in config]
    if missing:
        print(
            f"Error: {config_path} is missing required key(s): "
            f"{', '.join(repr(k) for k in missing)}.",
            file=sys.stderr,
        )
        return False

    bounds = config['bounds']
    if not isinstance(bounds, dict):
        print(
            f"Error: 'bounds' in {config_path} must be a mapping with "
            "keys 'south', 'west', 'north', 'east'.",
            file=sys.stderr,
        )
        return False

    bounds_keys = ('south', 'west', 'north', 'east')
    missing_bounds = [k for k in bounds_keys if k not in bounds]
    if missing_bounds:
        print(
            f"Error: 'bounds' in {config_path} is missing key(s): "
            f"{', '.join(repr(k) for k in missing_bounds)}.",
            file=sys.stderr,
        )
        return False

    return True


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
    parser.add_argument(
        '--expected-world-name',
        help='Expected world_name from CMake. Validates against YAML.',
    )
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    if not _validate_config(config, args.config):
        sys.exit(1)

    bounds = config['bounds']
    world_name = config['world_name']
    grid_power = config.get('grid_power', 9)

    # Validate world_name matches what CMake expects
    if args.expected_world_name and world_name != args.expected_world_name:
        print(
            f"Error: world_name in {args.config} is {world_name!r} but "
            f"CMake expects {args.expected_world_name!r}. Update the YAML "
            f"or the add_gazebo_world() call in CMakeLists.txt.",
            file=sys.stderr,
        )
        sys.exit(1)

    gen_args = [
        '--bounds',
        f"{bounds['south']},{bounds['west']},{bounds['north']},{bounds['east']}",
        '--output-dir', args.output_dir,
        '--world-name', world_name,
    ]

    # Multi-tile or single-tile terrain
    tiles = config.get('tiles')
    if tiles:
        for tile in tiles:
            tile_str = (
                f"{tile['name']}:"
                f"{tile['south']},{tile['west']},"
                f"{tile['north']},{tile['east']}:"
                f"{tile['grid_power']}"
            )
            gen_args.extend(['--tile', tile_str])
    else:
        gen_args.extend(['--grid-power', str(grid_power)])

    # Camera config (optional)
    camera = config.get('camera', {})
    if 'far_scale' in camera:
        gen_args.extend(['--camera-far-scale', str(camera['far_scale'])])
    if 'direction' in camera:
        gen_args.extend(['--camera-direction', camera['direction']])

    # OSM enrichment (optional)
    if config.get('osm'):
        gen_args.append('--osm')
    if config.get('no_osm_buildings'):
        gen_args.append('--no-osm-buildings')

    # ETOPO bathymetry (optional)
    if config.get('fetch_etopo'):
        gen_args.append('--fetch-etopo')

    # Feature category filtering (optional)
    skip = config.get('skip_categories')
    if skip:
        if isinstance(skip, list):
            skip = ','.join(skip)
        gen_args.extend(['--skip-categories', skip])

    # Disable all features (optional)
    if config.get('no_features'):
        gen_args.append('--no-features')

    # Geometry simplification tolerance (optional)
    simplify = config.get('simplify_tolerance')
    if simplify is not None:
        gen_args.extend(['--simplify-tolerance', str(simplify)])

    # Wall segment budget (optional)
    max_segs = config.get('max_wall_segments')
    if max_segs is not None:
        gen_args.extend(['--max-wall-segments', str(max_segs)])

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
