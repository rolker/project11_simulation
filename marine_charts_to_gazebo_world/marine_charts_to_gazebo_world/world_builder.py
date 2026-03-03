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

"""Generate Gazebo Harmonic SDF world files."""

import os


_TEMPLATE_PATH = os.path.join(
    os.path.dirname(__file__), "world_template.sdf.xml"
)


def generate_world_sdf(
    world_name: str,
    center_lat: float,
    center_lon: float,
    output_dir: str,
    heightmap_info: dict,
) -> str:
    """Generate a complete Gazebo Harmonic world SDF file.

    The world includes:
    - Spherical coordinates (WGS84, ENU)
    - DART physics (4ms step)
    - Standard Gazebo system plugins
    - Terrain heightmap model
    - Semi-transparent water surface plane
    - Scene with sky and lighting

    Args:
        world_name: Name for the world (used in filename and SDF).
        center_lat: Center latitude of the region (WGS84 degrees).
        center_lon: Center longitude of the region (WGS84 degrees).
        output_dir: Directory to write the world SDF into.
        heightmap_info: dict from terrain_to_heightmap() with terrain params.

    Returns:
        Path to the generated SDF file.
    """
    with open(_TEMPLATE_PATH) as f:
        template = f.read()

    sdf_content = template.format(
        world_name=world_name,
        center_lat=center_lat,
        center_lon=center_lon,
        water_size_x=heightmap_info["size_x"],
        water_size_y=heightmap_info["size_y"],
    )

    sdf_path = os.path.join(output_dir, f"{world_name}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_content)

    return sdf_path
