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

"""Generate Gazebo-compatible heightmap PNG from terrain data."""

import os

import numpy as np
from PIL import Image


def terrain_to_heightmap(
    terrain: np.ndarray,
    terrain_info: dict,
    output_dir: str,
    model_name: str = "terrain",
) -> dict:
    """Convert a terrain elevation grid to a 16-bit PNG heightmap.

    Gazebo heightmaps use 16-bit grayscale PNGs where pixel values map
    linearly to elevation. The grid must be (2^n + 1) x (2^n + 1).

    Args:
        terrain: 2D numpy array of elevation (meters, positive up).
        terrain_info: dict from build_terrain() with size and range info.
        output_dir: Directory to write heightmap files into.
        model_name: Gazebo model name for the terrain. Must be globally
            unique across all worlds (e.g. 'portsmouth_nh_harbor_terrain').

    Returns:
        dict with keys needed for SDF generation:
            'heightmap_path': path to the PNG file
            'model_name': the Gazebo model name used
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

    # Create model directory directly under output_dir so Gazebo can find it
    # via GZ_SIM_RESOURCE_PATH=<output_dir> as model://<model_name>
    model_dir = os.path.join(output_dir, model_name)
    os.makedirs(model_dir, exist_ok=True)

    # Save heightmap PNG
    heightmap_path = os.path.join(model_dir, "heightmap.png")
    img = Image.fromarray(normalized, mode="I;16")
    img.save(heightmap_path)

    # Generate terrain textures required by OGRE2's heightmap shader
    _write_terrain_textures(model_dir, terrain_clean, terrain_info)

    # Write model.config
    _write_model_config(model_dir, model_name)

    # Write model.sdf
    heightmap_info = {
        "heightmap_path": heightmap_path,
        "model_name": model_name,
        "size_x": terrain_info["size_x"],
        "size_y": terrain_info["size_y"],
        "size_z": elev_range,
        "pos_z": min_elev,
        "min_elevation": min_elev,
        "max_elevation": max_elev,
    }
    _write_model_sdf(model_dir, heightmap_info)

    return heightmap_info


def _write_terrain_textures(model_dir: str, terrain: np.ndarray,
                            terrain_info: dict):
    """Generate diffuse and normal map textures for the heightmap.

    OGRE2's terrain shader requires at least one texture layer in the
    visual heightmap; without it the generated shader fails to compile.
    Normal maps are computed from the terrain gradient for proper shading.
    """
    textures_dir = os.path.join(model_dir, "textures")
    os.makedirs(textures_dir, exist_ok=True)

    # Seafloor diffuse: muted sandy brown
    diffuse = Image.new("RGB", (16, 16), (160, 145, 120))
    diffuse.save(os.path.join(textures_dir, "seafloor_diffuse.png"))

    # Land diffuse: muted green-brown
    land = Image.new("RGB", (16, 16), (120, 140, 95))
    land.save(os.path.join(textures_dir, "land_diffuse.png"))

    # Compute normal map from terrain gradients
    # Pixel spacing in meters
    dx = terrain_info["size_x"] / (terrain.shape[1] - 1)
    dy = terrain_info["size_y"] / (terrain.shape[0] - 1)

    # Sobel-like gradient: dz/dx and dz/dy
    # Use numpy gradient which handles edges with one-sided differences
    gy, gx = np.gradient(terrain, dy, dx)

    # Normal vector: (-dz/dx, -dz/dy, 1), then normalize
    nx = -gx
    ny = -gy
    nz = np.ones_like(nx)
    length = np.sqrt(nx**2 + ny**2 + nz**2)
    nx /= length
    ny /= length
    nz /= length

    # Encode to tangent-space normal map: [-1,1] -> [0,255]
    normal_r = ((nx * 0.5 + 0.5) * 255).astype(np.uint8)
    normal_g = ((ny * 0.5 + 0.5) * 255).astype(np.uint8)
    normal_b = ((nz * 0.5 + 0.5) * 255).astype(np.uint8)

    normal_rgb = np.stack([normal_r, normal_g, normal_b], axis=-1)
    normal_img = Image.fromarray(normal_rgb, mode="RGB")
    normal_img.save(os.path.join(textures_dir, "terrain_normal.png"))


def _write_model_config(model_dir: str, model_name: str):
    """Write a Gazebo model.config for the terrain model."""
    config = f"""\
<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Generated terrain heightmap from S57/bathymetry data</description>
</model>
"""
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(config)


def _write_model_sdf(model_dir: str, info: dict):
    """Write the terrain model SDF with heightmap visual and collision."""
    # Blend height: transition from seafloor to land texture.
    # Expressed relative to the heightmap's pos_z (min elevation).
    # Sea level is at 0m, so offset from min = -min_elevation.
    blend_height = -info["pos_z"]

    mn = info['model_name']
    sdf = f"""\
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{mn}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <heightmap>
            <use_terrain_paging>false</use_terrain_paging>
            <texture>
              <diffuse>model://{mn}/textures/seafloor_diffuse.png</diffuse>
              <normal>model://{mn}/textures/terrain_normal.png</normal>
              <size>10</size>
            </texture>
            <texture>
              <diffuse>model://{mn}/textures/land_diffuse.png</diffuse>
              <normal>model://{mn}/textures/terrain_normal.png</normal>
              <size>10</size>
            </texture>
            <blend>
              <min_height>{blend_height:.1f}</min_height>
              <fade_dist>2</fade_dist>
            </blend>
            <uri>model://{mn}/heightmap.png</uri>
            <size>{info['size_x']:.1f} {info['size_y']:.1f} {info['size_z']:.1f}</size>
            <pos>0 0 {info['pos_z']:.1f}</pos>
          </heightmap>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <heightmap>
            <uri>model://{mn}/heightmap.png</uri>
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
