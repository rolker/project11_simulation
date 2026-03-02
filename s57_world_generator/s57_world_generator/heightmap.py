"""Generate Gazebo-compatible heightmap PNG from terrain data."""

import os

import numpy as np
from PIL import Image


def terrain_to_heightmap(
    terrain: np.ndarray,
    terrain_info: dict,
    output_dir: str,
) -> dict:
    """Convert a terrain elevation grid to a 16-bit PNG heightmap.

    Gazebo heightmaps use 16-bit grayscale PNGs where pixel values map
    linearly to elevation. The grid must be (2^n + 1) x (2^n + 1).

    Args:
        terrain: 2D numpy array of elevation (meters, positive up).
        terrain_info: dict from build_terrain() with size and range info.
        output_dir: Directory to write heightmap files into.

    Returns:
        dict with keys needed for SDF generation:
            'heightmap_path': path to the PNG file
            'size_x': terrain width in meters
            'size_y': terrain depth in meters
            'size_z': elevation range in meters
            'pos_z': vertical offset (min elevation)
            'min_elevation': minimum elevation value
            'max_elevation': maximum elevation value
    """
    grid_size = terrain.shape[0]
    if terrain.shape[0] != terrain.shape[1]:
        raise ValueError(f"Heightmap must be square, got {terrain.shape}")
    # Verify (2^n + 1) constraint
    n = grid_size - 1
    if not (n > 0 and (n & (n - 1)) == 0):
        raise ValueError(f"Grid size must be 2^n + 1, got {grid_size}")

    # Handle NaN values (fill with minimum elevation)
    min_elev = float(np.nanmin(terrain))
    max_elev = float(np.nanmax(terrain))
    terrain_clean = np.nan_to_num(terrain, nan=min_elev)

    # Normalize to 0..65535 range for 16-bit PNG
    elev_range = max_elev - min_elev
    if elev_range < 0.01:
        # Flat terrain — set to mid-range
        normalized = np.full_like(terrain_clean, 32768, dtype=np.uint16)
        elev_range = 1.0  # Avoid zero range in SDF
    else:
        normalized = ((terrain_clean - min_elev) / elev_range * 65535).astype(
            np.uint16
        )

    # Create model directory
    model_dir = os.path.join(output_dir, "models", "terrain")
    os.makedirs(model_dir, exist_ok=True)

    # Save heightmap PNG
    heightmap_path = os.path.join(model_dir, "heightmap.png")
    img = Image.fromarray(normalized, mode="I;16")
    img.save(heightmap_path)

    # Write model.config
    _write_model_config(model_dir)

    # Write model.sdf
    heightmap_info = {
        "heightmap_path": heightmap_path,
        "size_x": terrain_info["size_x"],
        "size_y": terrain_info["size_y"],
        "size_z": elev_range,
        "pos_z": min_elev,
        "min_elevation": min_elev,
        "max_elevation": max_elev,
    }
    _write_model_sdf(model_dir, heightmap_info)

    return heightmap_info


def _write_model_config(model_dir: str):
    """Write a Gazebo model.config for the terrain model."""
    config = """\
<?xml version="1.0"?>
<model>
  <name>terrain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Generated terrain heightmap from S57/bathymetry data</description>
</model>
"""
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(config)


def _write_model_sdf(model_dir: str, info: dict):
    """Write the terrain model SDF with heightmap visual and collision."""
    sdf = f"""\
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="terrain">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <heightmap>
            <uri>file://heightmap.png</uri>
            <size>{info['size_x']:.1f} {info['size_y']:.1f} {info['size_z']:.1f}</size>
            <pos>0 0 {info['pos_z']:.1f}</pos>
          </heightmap>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <heightmap>
            <uri>file://heightmap.png</uri>
            <size>{info['size_x']:.1f} {info['size_y']:.1f} {info['size_z']:.1f}</size>
            <pos>0 0 {info['pos_z']:.1f}</pos>
          </heightmap>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(sdf)
