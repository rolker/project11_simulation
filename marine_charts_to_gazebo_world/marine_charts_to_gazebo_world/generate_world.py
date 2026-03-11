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
from .coordinates import latlon_to_enu
from .feature_models import generate_feature_models
from .heightmap import terrain_to_heightmap
from .osm_fetcher import fetch_osm_features
from .osm_matcher import match_and_enrich
from .osm_texture import rasterize_osm_texture
from .s57_reader import BoundingBox, read_enc_directory
from .terrain import build_terrain
from .world_builder import generate_world_sdf


def _parse_bbox(s):
    """Parse 'south,west,north,east' string to BoundingBox."""
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 4:
        raise ValueError
    return BoundingBox(south=parts[0], west=parts[1],
                       north=parts[2], east=parts[3])


def _parse_tile(s):
    """Parse 'name:south,west,north,east:grid_power' string."""
    parts = s.split(":")
    if len(parts) != 3:
        raise ValueError(f"Tile must be name:bounds:power, got: {s}")
    name = parts[0]
    bbox = _parse_bbox(parts[1])
    grid_power = int(parts[2])
    return name, bbox, grid_power


def _bbox_center_enu(tile_bbox, world_center_lat, world_center_lon):
    """Compute ENU offset of a tile's center from the world center."""
    return latlon_to_enu(
        tile_bbox.center_lat, tile_bbox.center_lon,
        world_center_lat, world_center_lon,
    )


def _bbox_size_meters(bbox):
    """Compute size of a bbox in meters using ECEF-based ENU."""
    e_sw, n_sw = latlon_to_enu(
        bbox.south, bbox.west, bbox.center_lat, bbox.center_lon,
    )
    e_ne, n_ne = latlon_to_enu(
        bbox.north, bbox.east, bbox.center_lat, bbox.center_lon,
    )
    return e_ne - e_sw, n_ne - n_sw


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate Gazebo Harmonic SDF worlds from S57 ENC data.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Example (single tile):
              generate_world \\
                --enc-root /path/to/ENC_ROOT \\
                --bounds 43.065,-70.72,43.085,-70.70 \\
                --output-dir /tmp/portsmouth \\
                --world-name portsmouth_harbor

            Example (multi-tile):
              generate_world \\
                --bounds 42.98,-70.76,43.085,-70.60 \\
                --tile harbor:43.055,-70.76,43.085,-70.70:10 \\
                --tile ocean:42.98,-70.76,43.055,-70.60:9 \\
                --output-dir /tmp/portsmouth \\
                --world-name portsmouth_harbor
        """),
    )
    parser.add_argument(
        "--enc-root",
        default=os.environ.get("ROS_S57_ENC_ROOT"),
        help="Root directory containing S57 ENC .000 files. "
        "Defaults to $ROS_S57_ENC_ROOT if set.",
    )
    parser.add_argument(
        "--bounds",
        required=True,
        help="Overall bounding box as south,west,north,east in decimal degrees. "
        "Used for S57/OSM features and water plane.",
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
        help="Heightmap grid size as 2^N + 1 (default: 9 = 513x513). "
        "Used when no --tile args are given.",
    )
    parser.add_argument(
        "--tile",
        action="append",
        metavar="NAME:S,W,N,E:POWER",
        help="Terrain tile as name:south,west,north,east:grid_power. "
        "Can be specified multiple times for multi-tile worlds.",
    )
    parser.add_argument(
        "--cache-dir",
        help="Directory to cache downloaded bathymetry data.",
    )
    parser.add_argument(
        "--camera-far-scale",
        type=float,
        default=None,
        help="Far clipping plane as multiple of largest world dimension "
        "(default: 20).",
    )
    parser.add_argument(
        "--camera-direction",
        default=None,
        choices=["north", "south", "east", "west"],
        help="Initial camera viewing direction (default: north).",
    )
    parser.add_argument(
        "--fetch-etopo",
        action="store_true",
        help="Fetch ETOPO online bathymetry as base elevation. "
        "By default, terrain is built from S57 data only (land ramp + soundings).",
    )
    parser.add_argument(
        "--no-features",
        action="store_true",
        help="Disable S57 feature placement (buildings, buoys, etc.).",
    )
    parser.add_argument(
        "--debug-features",
        action="store_true",
        help="Use thin (0.2m) extrusions with distinct colors per feature "
        "type for footprint visualization.",
    )
    parser.add_argument(
        "--osm",
        action="store_true",
        help="Enrich S57 buildings with OpenStreetMap data (heights, "
        "materials, colours) and rasterize terrain textures via the "
        "Overpass API.",
    )
    parser.add_argument(
        "--no-osm-buildings",
        action="store_true",
        help="When used with --osm, skip adding unmatched OSM-only "
        "buildings. Only enrich existing S57 buildings and generate "
        "terrain textures.",
    )
    parser.add_argument(
        "--skip-categories",
        help="Comma-separated list of feature categories to skip. "
        "Valid categories: buildings, pontoons, bridges, buoys, beacons, "
        "lights, shore_constructions, piles, mooring_facilities, cranes, "
        "pylons.",
    )
    parser.add_argument(
        "--simplify-tolerance",
        type=float,
        default=None,
        help="Geometry simplification tolerance in degrees. "
        "If not set, derived from the world bounding box "
        "(~1m resolution for the smaller dimension).",
    )
    parser.add_argument(
        "--max-wall-segments",
        type=int,
        default=None,
        help="Maximum number of wall segments for shore constructions. "
        "Limits the total SLCONS segment count across all features.",
    )
    return parser.parse_args(argv)


def _build_single_tile(args, bbox, grid_size, s57_features,
                       ref_lat, ref_lon, model_name="terrain",
                       land_texture=None):
    """Build terrain and heightmap for a single tile.

    Returns (terrain_array, terrain_info, heightmap_info).
    """
    # Fetch online bathymetry (opt-in)
    elevation = None
    metadata = None
    if args.fetch_etopo:
        print(f"  Fetching ETOPO for {model_name}...")
        try:
            elevation, metadata = fetch_etopo(bbox, cache_dir=args.cache_dir)
            print(f"    Downloaded {elevation.shape[0]}x{elevation.shape[1]} grid")
        except Exception as e:
            print(f"    Warning: ETOPO download failed: {e}", file=sys.stderr)

    print(f"  Building terrain ({grid_size}x{grid_size})...")
    terrain, terrain_info = build_terrain(
        bbox=bbox,
        ref_lat=ref_lat,
        ref_lon=ref_lon,
        base_elevation=elevation,
        base_geotransform=metadata["geotransform"] if metadata else None,
        s57_features=s57_features,
        grid_size=grid_size,
    )
    print(
        f"    Range: {terrain_info['min_elevation']:.1f}m "
        f"to {terrain_info['max_elevation']:.1f}m"
    )

    print(f"  Generating heightmap for {model_name}...")
    heightmap_info = terrain_to_heightmap(
        terrain, terrain_info, args.output_dir, model_name=model_name,
        land_texture=land_texture,
    )
    print(f"    Saved to {heightmap_info['heightmap_path']}")

    return terrain, terrain_info, heightmap_info


def main(argv=None):
    args = parse_args(argv)

    # Parse overall bounding box
    try:
        bbox = _parse_bbox(args.bounds)
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

    # Parse tile specs (or use bounds as single tile)
    tiles = []
    if args.tile:
        for tile_str in args.tile:
            try:
                name, tile_bbox, grid_power = _parse_tile(tile_str)
                if grid_power < 1 or grid_power > 14:
                    raise ValueError("grid_power out of range")
                tiles.append((name, tile_bbox, grid_power))
            except ValueError as e:
                print(f"Error parsing --tile: {e}", file=sys.stderr)
                return 1
    else:
        if args.grid_power < 1 or args.grid_power > 14:
            print(
                "Error: --grid-power must be between 1 and 14",
                file=sys.stderr,
            )
            return 1
        tiles.append(("terrain", bbox, args.grid_power))

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
        n_features = sum([
            len(s57_features.buildings), len(s57_features.pontoons),
            len(s57_features.bridges), len(s57_features.buoys),
            len(s57_features.beacons), len(s57_features.lights),
            len(s57_features.shore_constructions),
            len(s57_features.piles),
            len(s57_features.mooring_facilities),
            len(s57_features.cranes), len(s57_features.pylons),
        ])
        if n_features > 0:
            print(
                f"  Found {len(s57_features.buildings)} buildings, "
                f"{len(s57_features.pontoons)} pontoons, "
                f"{len(s57_features.bridges)} bridges, "
                f"{len(s57_features.buoys)} buoys, "
                f"{len(s57_features.beacons)} beacons, "
                f"{len(s57_features.lights)} lights, "
                f"{len(s57_features.shore_constructions)} shore constructions, "
                f"{len(s57_features.piles)} piles, "
                f"{len(s57_features.mooring_facilities)} mooring facilities, "
                f"{len(s57_features.cranes)} cranes, "
                f"{len(s57_features.pylons)} pylons"
            )

    # Step 1.5: OSM enrichment (optional)
    osm_features = None
    if args.osm:
        try:
            print("Fetching OSM data (with terrain features)...")
            osm_features = fetch_osm_features(
                bbox, cache_dir=args.cache_dir,
                fetch_terrain_features=True,
            )
            print(f"  Found {len(osm_features.buildings)} OSM buildings, "
                  f"{len(osm_features.landuse)} landuse, "
                  f"{len(osm_features.roads)} roads, "
                  f"{len(osm_features.parking)} parking, "
                  f"{len(osm_features.natural)} natural areas")
            if s57_features is not None:
                s57_features, n_matched, n_added = match_and_enrich(
                    s57_features, osm_features,
                    add_unmatched=not args.no_osm_buildings,
                )
                print(f"  Matched {n_matched} S57 buildings with OSM data")
                print(f"  Added {n_added} OSM-only buildings")
        except Exception as e:
            print(f"  Warning: OSM enrichment failed, skipping: {e}")
            osm_features = None

    # Step 2-4: Build terrain tiles
    ref_lat = bbox.center_lat
    ref_lon = bbox.center_lon
    multi_tile = len(tiles) > 1
    first_terrain = None
    first_terrain_info = None

    if multi_tile:
        print(f"Building {len(tiles)} terrain tiles...")
        tile_results = []  # list of heightmap_info dicts (with pos_x/pos_y set)
        for name, tile_bbox, grid_power in tiles:
            grid_size = 2 ** grid_power + 1
            print(f"\nTile '{name}' ({grid_size}x{grid_size}):")

            # Rasterize OSM texture for this tile
            land_texture = None
            if osm_features is not None:
                tile_size_x, tile_size_y = _bbox_size_meters(tile_bbox)
                tile_terrain_info = {
                    "size_x": tile_size_x, "size_y": tile_size_y,
                }
                print("  Rasterizing OSM terrain texture...")
                land_texture = rasterize_osm_texture(
                    osm_features, tile_terrain_info, grid_size,
                    ref_lat, ref_lon,
                )

            terrain, terrain_info, heightmap_info = _build_single_tile(
                args, tile_bbox, grid_size, s57_features,
                ref_lat=ref_lat, ref_lon=ref_lon,
                model_name=f"terrain_{name}",
                land_texture=land_texture,
            )
            # Embed ENU offset directly in the heightmap <pos> element
            # (Gazebo's OGRE2 heightmap renderer uses <pos>, not model pose)
            enu_x, enu_y = _bbox_center_enu(
                tile_bbox, ref_lat, ref_lon,
            )
            heightmap_info["pos_x"] = enu_x
            heightmap_info["pos_y"] = enu_y
            # Rewrite model.sdf with updated position
            from .heightmap import _write_model_sdf
            model_dir = os.path.join(args.output_dir, heightmap_info["model_name"])
            _write_model_sdf(model_dir, heightmap_info)
            print(f"    ENU offset: ({enu_x:.1f}, {enu_y:.1f})m")
            tile_results.append(heightmap_info)
            if first_terrain is None:
                first_terrain = terrain
                first_terrain_info = terrain_info
        heightmap_result = tile_results
    else:
        name, tile_bbox, grid_power = tiles[0]
        grid_size = 2 ** grid_power + 1
        if not args.fetch_etopo:
            print("Using S57-only terrain (land ramp + soundings).")

        # Rasterize OSM texture for the single tile
        land_texture = None
        if osm_features is not None:
            tile_size_x, tile_size_y = _bbox_size_meters(tile_bbox)
            tile_terrain_info = {
                "size_x": tile_size_x, "size_y": tile_size_y,
            }
            print("Rasterizing OSM terrain texture...")
            land_texture = rasterize_osm_texture(
                osm_features, tile_terrain_info, grid_size,
                ref_lat, ref_lon,
            )
            n_features = (len(osm_features.landuse) + len(osm_features.roads)
                          + len(osm_features.parking) + len(osm_features.natural))
            print(f"  Rasterized {n_features} terrain features "
                  f"onto {grid_size}x{grid_size} texture")

        first_terrain, first_terrain_info, heightmap_info = _build_single_tile(
            args, tile_bbox, grid_size, s57_features,
            ref_lat=ref_lat, ref_lon=ref_lon,
            model_name=f"{args.world_name}_terrain",
            land_texture=land_texture,
        )
        heightmap_result = heightmap_info

    # Step 5: Generate feature models (optional)
    s57_feature_sdf = ""
    osm_feature_sdf = ""
    skip_categories = set()
    if args.skip_categories:
        skip_categories = {c.strip() for c in args.skip_categories.split(',')}

    # Compute simplification tolerance from world scale if not specified.
    # Default: ~1m expressed in degrees (via the smaller bbox dimension).
    if args.simplify_tolerance is not None:
        simplify_tol = args.simplify_tolerance
    else:
        min_span = min(bbox.north - bbox.south, bbox.east - bbox.west)
        simplify_tol = min_span / 10000.0  # ~1m for typical coastal regions

    # Compute terrain bounds in ENU for feature elevation sampling
    terrain_bounds = None
    if first_terrain is not None:
        e_sw, n_sw = latlon_to_enu(bbox.south, bbox.west, ref_lat, ref_lon)
        e_ne, n_ne = latlon_to_enu(bbox.north, bbox.east, ref_lat, ref_lon)
        terrain_bounds = {
            'min_east': e_sw,
            'max_east': e_ne,
            'min_north': n_sw,
            'max_north': n_ne,
        }

    if not args.no_features and s57_features is not None:
        print("Generating feature models...")
        feature_groups = generate_feature_models(
            s57_features, ref_lat, ref_lon,
            terrain=first_terrain, terrain_bounds=terrain_bounds,
            debug=args.debug_features,
            skip_categories=skip_categories,
            simplify_tolerance=simplify_tol,
            max_wall_segments=args.max_wall_segments,
        )
        s57_feature_sdf = feature_groups['s57']
        osm_feature_sdf = feature_groups['osm']
        total = s57_feature_sdf + osm_feature_sdf
        if total:
            n_models = total.count("<model name=")
            print(f"  Generated {n_models} feature models")
        else:
            print("  No placeable features found")

    # Step 6: Generate world SDF
    camera_config = {}
    if args.camera_far_scale is not None:
        camera_config["far_scale"] = args.camera_far_scale
    if args.camera_direction is not None:
        camera_config["direction"] = args.camera_direction

    water_x, water_y = _bbox_size_meters(bbox)
    print("Generating world SDF...")
    sdf_path = generate_world_sdf(
        world_name=args.world_name,
        center_lat=bbox.center_lat,
        center_lon=bbox.center_lon,
        output_dir=args.output_dir,
        heightmap_info=heightmap_result,
        camera_config=camera_config or None,
        s57_feature_models=s57_feature_sdf,
        osm_feature_models=osm_feature_sdf,
        water_size_x=water_x,
        water_size_y=water_y,
    )
    print(f"  World SDF saved to {sdf_path}")

    # Step 7: Write generation metadata
    _write_readme(args, bbox, first_terrain_info, heightmap_result)

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
        if isinstance(heightmap_info, list):
            f.write(f"- **Tiles**: {len(heightmap_info)}\n")
            for info in heightmap_info:
                name = info.get('model_name', 'terrain')
                f.write(
                    f"  - {name}: offset "
                    f"({info.get('pos_x', 0):.0f}, "
                    f"{info.get('pos_y', 0):.0f})m\n"
                )
        else:
            f.write(f"- **Grid size**: {terrain_info['grid_size']}x"
                    f"{terrain_info['grid_size']}\n")
            f.write(f"- **Terrain extent**: {terrain_info['size_x']:.0f}m x "
                    f"{terrain_info['size_y']:.0f}m\n")
            min_e = heightmap_info['min_elevation']
            max_e = heightmap_info['max_elevation']
            f.write(f"- **Elevation range**: {min_e:.1f}m to {max_e:.1f}m\n")


if __name__ == "__main__":
    sys.exit(main())
