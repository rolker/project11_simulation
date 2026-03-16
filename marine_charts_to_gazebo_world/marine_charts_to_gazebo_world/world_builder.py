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

import math
import os


_TEMPLATE_PATH = os.path.join(
    os.path.dirname(__file__), "world_template.sdf.xml"
)

# Supported cardinal directions for the initial camera view.
# In Gazebo ENU, the camera default forward is +X (East), so:
#   yaw = 0       -> facing East
#   yaw = pi/2    -> facing North
#   yaw = pi      -> facing West
#   yaw = -pi/2   -> facing South
# Each entry: (yaw_radians, x_offset_sign, y_offset_sign)
# The offset signs place the camera behind the viewing direction.
_CAMERA_DIRECTIONS = {
    "north": (math.pi / 2, 0, -1),
    "south": (-math.pi / 2, 0, 1),
    "east": (0.0, -1, 0),
    "west": (math.pi, 1, 0),
}


def _compute_camera(heightmap_info, camera_config=None):
    """Compute camera pose and far plane from world size and config.

    Args:
        heightmap_info: dict with 'size_x' and 'size_y' (meters).
        camera_config: optional dict with overrides:
            'far_scale': multiplier on max dimension (default 20)
            'direction': 'north', 'south', 'east', 'west' (default 'north')
            'pose': explicit 'x y z roll pitch yaw' override (skips computation)
            'far': explicit far plane in meters (skips computation)

    Returns:
        (camera_pose_str, camera_far_meters)
    """
    cfg = camera_config or {}
    size_x = heightmap_info["size_x"]
    size_y = heightmap_info["size_y"]
    max_dim = max(size_x, size_y)

    # Far plane
    if "far" in cfg:
        camera_far = float(cfg["far"])
    else:
        far_scale = float(cfg.get("far_scale", 20))
        camera_far = max_dim * far_scale

    # Camera pose
    if "pose" in cfg:
        camera_pose = cfg["pose"]
    else:
        direction = cfg.get("direction", "north")
        if direction not in _CAMERA_DIRECTIONS:
            raise ValueError(
                f"Unknown camera direction {direction!r}, "
                f"must be one of {list(_CAMERA_DIRECTIONS.keys())}"
            )
        yaw, x_sign, y_sign = _CAMERA_DIRECTIONS[direction]

        # Position camera so most of the world is visible.
        # Height ~0.7x max dimension gives a good overview.
        height = max_dim * 0.7
        pitch = 0.8  # ~46 degrees down from horizontal

        # Offset the camera behind the viewing direction so the view
        # center hits approximately the world origin (terrain center).
        # offset = height / tan(pitch) centers the view on the ground.
        offset = height / math.tan(pitch)
        cx = x_sign * offset
        cy = y_sign * offset

        camera_pose = f"{cx:.0f} {cy:.0f} {height:.0f} 0 {pitch} {yaw:.4f}"

    return camera_pose, camera_far


def _build_terrain_includes(heightmap_info):
    """Build SDF <include> elements for terrain tiles.

    Args:
        heightmap_info: Either a single dict or a list of dicts for multi-tile.
            ENU offsets are embedded in each tile's heightmap <pos> element,
            not in the <include> pose (Gazebo's OGRE2 heightmap renderer
            uses <pos> directly, ignoring model pose).

    Returns:
        String of SDF <include> elements.
    """
    if isinstance(heightmap_info, dict):
        model_name = heightmap_info.get("model_name", "terrain")
        return f'    <include>\n      <uri>{model_name}</uri>\n    </include>'

    # Multiple tiles — position is already in each model's heightmap <pos>
    includes = []
    for info in heightmap_info:
        model_name = info.get("model_name", "terrain")
        includes.append(
            f'    <include>\n'
            f'      <uri>{model_name}</uri>\n'
            f'    </include>'
        )
    return '\n'.join(includes)


# Default wavefield parameters matching VRX conventions.
_WAVE_DEFAULTS = {
    "gain": 0.3,
    "period": 5.0,
    "direction": 0.0,
    "steepness": 0.0,
    "topic": "/vrx/wavefield/parameters",
}


def _build_wave_sdf(wave_config):
    """Build SDF for VRX wave visuals and wavefield parameter publisher.

    Args:
        wave_config: dict with optional keys 'gain', 'period', 'direction',
            'steepness', 'topic'. Missing keys use defaults.

    Returns:
        String of SDF XML (coast_waves include + PublisherPlugin).
    """
    cfg = {**_WAVE_DEFAULTS, **(wave_config or {})}
    topic = cfg["topic"]

    lines = [
        '    <!-- VRX wave visuals (Gerstner waves) -->',
        '    <include>',
        '      <uri>coast_waves</uri>',
        '    </include>',
        '',
        '    <!-- Wavefield parameter publisher for buoyancy plugins -->',
        '    <plugin filename="libPublisherPlugin.so"',
        '            name="vrx::PublisherPlugin">',
        f'      <message type="gz.msgs.Param" topic="{topic}"',
        '               every="2.0">',
    ]

    for key in ("direction", "gain", "period", "steepness"):
        lines.extend([
            '        params {',
            f'          key: "{key}"',
            '          value {',
            '            type: DOUBLE',
            f'            double_value: {cfg[key]}',
            '          }',
            '        }',
        ])

    lines.extend([
        '      </message>',
        '    </plugin>',
    ])

    return '\n'.join(lines)


def generate_world_sdf(
    world_name: str,
    center_lat: float,
    center_lon: float,
    output_dir: str,
    heightmap_info,
    camera_config: dict = None,
    s57_feature_models: str = "",
    osm_feature_models: str = "",
    water_size_x: float = 0.0,
    water_size_y: float = 0.0,
    wave_config: dict = None,
) -> str:
    """Generate a complete Gazebo Harmonic world SDF file.

    The world includes:
    - Spherical coordinates (WGS84, ENU)
    - DART physics (4ms step)
    - Standard Gazebo system plugins
    - Terrain heightmap model(s)
    - Semi-transparent water surface plane
    - Scene with sky and lighting
    - Optional S57/OSM chart feature models (buildings, buoys, etc.)
    - Optional VRX wave visuals and wavefield parameter publisher

    Args:
        world_name: Name for the world (used in filename and SDF).
        center_lat: Center latitude of the region (WGS84 degrees).
        center_lon: Center longitude of the region (WGS84 degrees).
        output_dir: Directory to write the world SDF into.
        heightmap_info: dict from terrain_to_heightmap() for single tile,
            or list of dicts for multi-tile.
        camera_config: optional dict with camera overrides (see _compute_camera).
        s57_feature_models: SDF XML for S57-sourced features (default empty).
        osm_feature_models: SDF XML for OSM-sourced features (default empty).
        water_size_x: Override water plane width (meters). If 0, uses
            heightmap_info size.
        water_size_y: Override water plane height (meters). If 0, uses
            heightmap_info size.
        wave_config: optional dict with wave parameter overrides. When not
            None, VRX coast_waves visuals and a wavefield PublisherPlugin
            are added. Keys: gain, period, direction, steepness, topic.

    Returns:
        Path to the generated SDF file.
    """
    # Determine water size
    if water_size_x > 0 and water_size_y > 0:
        wx, wy = water_size_x, water_size_y
    elif isinstance(heightmap_info, dict):
        wx = heightmap_info["size_x"]
        wy = heightmap_info["size_y"]
    else:
        # Multi-tile: use the max extent
        wx = max(info["size_x"] for info in heightmap_info)
        wy = max(info["size_y"] for info in heightmap_info)

    # For camera computation, provide size info
    camera_size_info = {"size_x": wx, "size_y": wy}
    camera_pose, camera_far = _compute_camera(camera_size_info, camera_config)

    with open(_TEMPLATE_PATH) as f:
        template = f.read()

    terrain_includes = _build_terrain_includes(heightmap_info)
    wave_models = _build_wave_sdf(wave_config) if wave_config is not None else ""

    sdf_content = template.format(
        world_name=world_name,
        center_lat=center_lat,
        center_lon=center_lon,
        water_size_x=wx,
        water_size_y=wy,
        terrain_includes=terrain_includes,
        camera_pose=camera_pose,
        camera_far=camera_far,
        s57_feature_models=s57_feature_models,
        osm_feature_models=osm_feature_models,
        wave_models=wave_models,
    )

    sdf_path = os.path.join(output_dir, f"{world_name}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_content)

    return sdf_path
