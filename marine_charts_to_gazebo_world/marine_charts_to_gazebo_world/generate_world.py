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

"""CLI entry point for S57 world generation."""

import argparse
import os
import sys
import textwrap
from datetime import datetime

from .bathy_fetcher import fetch_etopo
from .heightmap import terrain_to_heightmap
from .s57_reader import BoundingBox, read_enc_directory
from .terrain import build_terrain
from .world_builder import generate_world_sdf


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate Gazebo Harmonic SDF worlds from S57 ENC data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Example:
              generate_world \\
                --enc-root /path/to/ENC_ROOT \\
                --bounds 43.065,-70.72,43.085,-70.70 \\
                --output-dir /tmp/portsmouth \\
                --world-name portsmouth_harbor
        """),
    )
    parser.add_argument(
        "--enc-root",
        help="Root directory containing S57 ENC .000 files. "
        "If omitted, only online bathymetry data is used.",
    )
    parser.add_argument(
        "--bounds",
        required=True,
        help="Bounding box as south,west,north,east in decimal degrees. "
        "Example: 43.065,-70.72,43.085,-70.70",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory to write generated world files into.",
    )
    parser.add_argument(
        "--world-name",
        required=True,
        help="Name for the generated world (used in filenames and SDF).",
    )
    parser.add_argument(
        "--grid-power",
        type=int,
        default=9,
        help="Heightmap grid size as 2^N + 1 (default: 9 = 513x513).",
    )
    parser.add_argument(
        "--cache-dir",
        help="Directory to cache downloaded bathymetry data.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # Parse bounding box
    try:
        parts = [float(x) for x in args.bounds.split(",")]
        if len(parts) != 4:
            raise ValueError
        bbox = BoundingBox(south=parts[0], west=parts[1],
                           north=parts[2], east=parts[3])
    except (ValueError, IndexError):
        print(
            "Error: --bounds must be four comma-separated decimal degree values: "
            "south,west,north,east",
            file=sys.stderr,
        )
        return 1

    if bbox.south >= bbox.north or bbox.west >= bbox.east:
        print(
            "Error: bounds must have south < north and west < east",
            file=sys.stderr,
        )
        return 1

    if args.grid_power < 1 or args.grid_power > 14:
        print(
            "Error: --grid-power must be between 1 and 14",
            file=sys.stderr,
        )
        return 1

    grid_size = 2 ** args.grid_power + 1
    os.makedirs(args.output_dir, exist_ok=True)

    # Step 1: Read S57 charts (optional)
    s57_features = None
    if args.enc_root:
        print(f"Reading S57 charts from {args.enc_root}...")
        s57_features = read_enc_directory(args.enc_root, bbox)
        print(
            f"  Found {len(s57_features.depth_areas)} depth areas, "
            f"{len(s57_features.soundings)} soundings, "
            f"{len(s57_features.land_areas)} land areas, "
            f"{len(s57_features.coastlines)} coastlines"
        )

    # Step 2: Fetch online bathymetry
    elevation = None
    metadata = None
    print("Fetching ETOPO elevation data...")
    try:
        elevation, metadata = fetch_etopo(bbox, cache_dir=args.cache_dir)
        print(f"  Downloaded {elevation.shape[0]}x{elevation.shape[1]} grid")
        print(
            f"  Elevation range: {elevation.min():.1f}m to "
            f"{elevation.max():.1f}m"
        )
    except Exception as e:
        print(f"  Warning: ETOPO download failed: {e}", file=sys.stderr)
        if s57_features is None or not s57_features.soundings:
            print(
                "Error: No elevation data available. Provide --enc-root with "
                "S57 charts or ensure network access for ETOPO download.",
                file=sys.stderr,
            )
            return 1
        print("  Falling back to S57-only terrain generation.")

    # Step 3: Build terrain surface
    print(f"Building terrain ({grid_size}x{grid_size})...")
    terrain, terrain_info = build_terrain(
        bbox=bbox,
        base_elevation=elevation,
        base_geotransform=metadata["geotransform"] if metadata else None,
        s57_features=s57_features,
        grid_size=grid_size,
    )
    print(
        f"  Terrain range: {terrain_info['min_elevation']:.1f}m "
        f"to {terrain_info['max_elevation']:.1f}m"
    )

    # Step 4: Generate heightmap
    terrain_model_name = f"{args.world_name}_terrain"
    print(f"Generating heightmap (model: {terrain_model_name})...")
    heightmap_info = terrain_to_heightmap(
        terrain, terrain_info, args.output_dir,
        model_name=terrain_model_name,
    )
    print(f"  Heightmap saved to {heightmap_info['heightmap_path']}")

    # Step 5: Generate world SDF
    print("Generating world SDF...")
    sdf_path = generate_world_sdf(
        world_name=args.world_name,
        center_lat=bbox.center_lat,
        center_lon=bbox.center_lon,
        output_dir=args.output_dir,
        heightmap_info=heightmap_info,
    )
    print(f"  World SDF saved to {sdf_path}")

    # Step 6: Write generation metadata
    _write_readme(args, bbox, terrain_info, heightmap_info)

    print(f"\nWorld generation complete: {args.output_dir}/")
    print(f"  To launch: GZ_SIM_RESOURCE_PATH={args.output_dir} "
          f"gz sim {args.world_name}.sdf")
    return 0


def _write_readme(args, bbox, terrain_info, heightmap_info):
    """Write a README with generation metadata."""
    readme_path = os.path.join(args.output_dir, "README.md")
    with open(readme_path, "w") as f:
        f.write(f"# {args.world_name}\n\n")
        f.write(f"Generated by `marine_charts_to_gazebo_world` on "
                f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
        f.write("## Parameters\n\n")
        f.write(f"- **Bounds**: {bbox.south:.6f}, {bbox.west:.6f}, "
                f"{bbox.north:.6f}, {bbox.east:.6f}\n")
        f.write(f"- **Center**: {bbox.center_lat:.6f}, "
                f"{bbox.center_lon:.6f}\n")
        if args.enc_root:
            f.write(f"- **ENC root**: `{args.enc_root}`\n")
        f.write(f"- **Grid size**: {terrain_info['grid_size']}x"
                f"{terrain_info['grid_size']}\n")
        f.write(f"- **Terrain extent**: {terrain_info['size_x']:.0f}m x "
                f"{terrain_info['size_y']:.0f}m\n")
        f.write(f"- **Elevation range**: {heightmap_info['min_elevation']:.1f}m "
                f"to {heightmap_info['max_elevation']:.1f}m\n")


if __name__ == "__main__":
    sys.exit(main())
