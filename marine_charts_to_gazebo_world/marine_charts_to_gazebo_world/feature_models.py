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

"""Convert S57 features to parametric SDF model elements for Gazebo."""

import math

from marine_autonomy.wgs84 import toECEFfromDegrees

# Default extrusion heights by OBJL code
_BUILDING_HEIGHTS = {
    12: 5.0,   # BUISGL — generic building
    73: 12.0,  # LNDMRK — landmark (taller)
    119: 8.0,  # SILTNK — silo/tank
}
_PONTOON_HEIGHT = 1.0
_BRIDGE_DEFAULT_HEIGHT = 10.0

# Debug mode colors per feature type (ambient, diffuse)
_DEBUG_COLORS = {
    'building': ('0.8 0.2 0.2 0.8', '0.9 0.3 0.3 0.8'),   # red
    'pontoon':  ('0.2 0.2 0.8 0.8', '0.3 0.3 0.9 0.8'),   # blue
    'bridge':   ('0.8 0.8 0.2 0.8', '0.9 0.9 0.3 0.8'),   # yellow
}

# S57 COLOUR attribute mapping to RGB
_S57_COLOURS = {
    1: (1.0, 1.0, 1.0),    # white
    2: (0.1, 0.1, 0.1),    # black
    3: (1.0, 0.0, 0.0),    # red
    4: (0.0, 0.8, 0.0),    # green
    5: (0.0, 0.0, 1.0),    # blue
    6: (1.0, 1.0, 0.0),    # yellow
}


def _latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """Convert WGS84 lat/lon to local ENU meters via ECEF."""
    rx, ry, rz = toECEFfromDegrees(ref_lat, ref_lon)
    px, py, pz = toECEFfromDegrees(lat, lon)
    dx, dy, dz = px - rx, py - ry, pz - rz
    lat_r = math.radians(ref_lat)
    lon_r = math.radians(ref_lon)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lon = math.sin(lon_r)
    cos_lon = math.cos(lon_r)
    east = -sin_lon * dx + cos_lon * dy
    north = (-sin_lat * cos_lon * dx
             - sin_lat * sin_lon * dy
             + cos_lat * dz)
    return east, north


def _sample_terrain_elevation(lat, lon, terrain, bbox):
    """Look up terrain elevation at a lat/lon via bilinear interpolation.

    Args:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        terrain: 2D numpy array, north-up (row 0 = north).
        bbox: BoundingBox with south/north/west/east.

    Returns:
        Elevation in meters, or 0.0 if outside bounds.
    """
    if terrain is None:
        return 0.0
    rows, cols = terrain.shape
    # Fractional column (west=0, east=cols-1)
    fc = (lon - bbox.west) / (bbox.east - bbox.west) * (cols - 1)
    # Fractional row (north=0, south=rows-1)
    fr = (bbox.north - lat) / (bbox.north - bbox.south) * (rows - 1)

    if fc < 0 or fc > cols - 1 or fr < 0 or fr > rows - 1:
        return 0.0

    c0 = int(math.floor(fc))
    r0 = int(math.floor(fr))
    c1 = min(c0 + 1, cols - 1)
    r1 = min(r0 + 1, rows - 1)
    dc = fc - c0
    dr = fr - r0

    return float(
        terrain[r0, c0] * (1 - dc) * (1 - dr)
        + terrain[r0, c1] * dc * (1 - dr)
        + terrain[r1, c0] * (1 - dc) * dr
        + terrain[r1, c1] * dc * dr
    )


def _polygon_to_polyline_model(
    name, geometry, center_lat, center_lon, height, z=0.0,
    ambient='0.6 0.6 0.55 1.0', diffuse='0.7 0.7 0.65 1.0',
):
    """Generate an SDF <model> with <polyline> extrusion for a polygon."""
    # Get the exterior ring of the polygon
    geom_type = geometry.GetGeometryType() & 0xFF
    if geom_type == 3:  # wkbPolygon
        ring = geometry.GetGeometryRef(0)
    elif geom_type == 6:  # wkbMultiPolygon
        # Use largest polygon
        largest = None
        largest_area = 0.0
        for i in range(geometry.GetGeometryCount()):
            sub = geometry.GetGeometryRef(i)
            area = sub.GetArea()
            if area > largest_area:
                largest_area = area
                largest = sub
        if largest is None:
            return ''
        ring = largest.GetGeometryRef(0)
    else:
        return ''

    if ring is None or ring.GetPointCount() < 3:
        return ''

    # Compute centroid in lat/lon
    centroid = geometry.Centroid()
    clat, clon = centroid.GetY(), centroid.GetX()
    cx, cy = _latlon_to_enu(clat, clon, center_lat, center_lon)

    # Convert ring points to local offsets relative to centroid
    points = []
    for i in range(ring.GetPointCount()):
        lon, lat = ring.GetX(i), ring.GetY(i)
        px, py = _latlon_to_enu(lat, lon, center_lat, center_lon)
        points.append(
            f'          <point>{px - cx:.2f} {py - cy:.2f}</point>'
        )

    points_str = '\n'.join(points)
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{cx:.2f} {cy:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <geometry>\n'
        f'            <polyline>\n'
        f'              <height>{height:.1f}</height>\n'
        f'{points_str}\n'
        f'            </polyline>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>{ambient}</ambient>\n'
        f'            <diffuse>{diffuse}</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'        <collision name="collision">\n'
        f'          <geometry>\n'
        f'            <polyline>\n'
        f'              <height>{height:.1f}</height>\n'
        f'{points_str}\n'
        f'            </polyline>\n'
        f'          </geometry>\n'
        f'        </collision>\n'
        f'      </link>\n'
        f'    </model>'
    )


def _buoy_model(name, x, y, colour_code):
    """Generate an SDF buoy model (cylinder body + cone top)."""
    r, g, b = _S57_COLOURS.get(colour_code, (1.0, 1.0, 0.0))
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="body">\n'
        f'          <pose>0 0 0.4 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.4</radius>\n'
        f'              <length>0.8</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>{r:.1f} {g:.1f} {b:.1f} 1.0</ambient>\n'
        f'            <diffuse>{r:.1f} {g:.1f} {b:.1f} 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'        <collision name="collision">\n'
        f'          <pose>0 0 0.4 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.4</radius>\n'
        f'              <length>0.8</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'        </collision>\n'
        f'        <visual name="top">\n'
        f'          <pose>0 0 1.0 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cone>\n'
        f'              <radius>0.2</radius>\n'
        f'              <length>0.4</length>\n'
        f'            </cone>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>{r:.1f} {g:.1f} {b:.1f} 1.0</ambient>\n'
        f'            <diffuse>{r:.1f} {g:.1f} {b:.1f} 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def _beacon_model(name, x, y, z=0.0):
    """Generate an SDF beacon model (tall gray cylinder)."""
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <pose>0 0 1.5 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.15</radius>\n'
        f'              <length>3.0</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.5 0.5 0.5 1.0</ambient>\n'
        f'            <diffuse>0.6 0.6 0.6 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'        <collision name="collision">\n'
        f'          <pose>0 0 1.5 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.15</radius>\n'
        f'              <length>3.0</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'        </collision>\n'
        f'      </link>\n'
        f'    </model>'
    )


def _light_model(name, x, y, z=0.0):
    """Generate an SDF light structure model (pole + yellow sphere on top)."""
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="pole">\n'
        f'          <pose>0 0 2.5 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.1</radius>\n'
        f'              <length>5.0</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.3 0.3 0.3 1.0</ambient>\n'
        f'            <diffuse>0.4 0.4 0.4 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'        <collision name="collision">\n'
        f'          <pose>0 0 2.5 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.1</radius>\n'
        f'              <length>5.0</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'        </collision>\n'
        f'        <visual name="lamp">\n'
        f'          <pose>0 0 5.15 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <sphere>\n'
        f'              <radius>0.15</radius>\n'
        f'            </sphere>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>1.0 1.0 0.0 1.0</ambient>\n'
        f'            <diffuse>1.0 1.0 0.0 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def generate_feature_models(
    features, center_lat, center_lon,
    terrain=None, bbox=None, debug=False,
):
    """Convert S57 features to SDF <model> XML strings.

    Args:
        features: S57Features instance with extracted chart features.
        center_lat: World center latitude (WGS84 degrees).
        center_lon: World center longitude (WGS84 degrees).
        terrain: Optional 2D numpy array of elevation (north-up, meters).
        bbox: Optional BoundingBox for terrain sampling.
        debug: If True, polygon features use thin (0.2m) extrusions with
            distinct colors per type for footprint visualization.

    Returns:
        String of SDF <model> elements ready to insert into a world template.
        Empty string if no features to generate.
    """
    models = []

    # Buildings (BUISGL, LNDMRK, SILTNK)
    for i, building in enumerate(features.buildings):
        centroid = building.geometry.Centroid()
        z = _sample_terrain_elevation(
            centroid.GetY(), centroid.GetX(), terrain, bbox
        )
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['building']
        else:
            height = _BUILDING_HEIGHTS.get(building.objl, 5.0)
            amb = '0.6 0.6 0.55 1.0'
            dif = '0.7 0.7 0.65 1.0'
        model = _polygon_to_polyline_model(
            f'building_{i:04d}', building.geometry,
            center_lat, center_lon, height, z=z,
            ambient=amb, diffuse=dif,
        )
        if model:
            models.append(model)

    # Pontoons (water surface, z=0)
    for i, pontoon in enumerate(features.pontoons):
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['pontoon']
        else:
            height = _PONTOON_HEIGHT
            amb = '0.4 0.35 0.3 1.0'
            dif = '0.5 0.45 0.4 1.0'
        model = _polygon_to_polyline_model(
            f'pontoon_{i:04d}', pontoon.geometry,
            center_lat, center_lon, height, z=0.0,
            ambient=amb, diffuse=dif,
        )
        if model:
            models.append(model)

    # Bridges (deck at clearance height, not extruded from 0)
    for i, bridge in enumerate(features.bridges):
        clearance = (bridge.clearance if bridge.clearance > 0
                     else _BRIDGE_DEFAULT_HEIGHT)
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['bridge']
        else:
            height = 1.0  # deck thickness
            amb = '0.5 0.5 0.5 1.0'
            dif = '0.6 0.6 0.6 1.0'
        model = _polygon_to_polyline_model(
            f'bridge_{i:04d}', bridge.geometry,
            center_lat, center_lon, height, z=clearance,
            ambient=amb, diffuse=dif,
        )
        if model:
            models.append(model)

    # Buoys (water surface, z=0)
    for i, buoy in enumerate(features.buoys):
        x, y = _latlon_to_enu(buoy.lat, buoy.lon, center_lat, center_lon)
        models.append(_buoy_model(f'buoy_{i:04d}', x, y, buoy.colour))

    # Beacons (on terrain)
    for i, beacon in enumerate(features.beacons):
        x, y = _latlon_to_enu(
            beacon.lat, beacon.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(
            beacon.lat, beacon.lon, terrain, bbox
        )
        models.append(_beacon_model(f'beacon_{i:04d}', x, y, z=z))

    # Lights (on terrain)
    for i, light in enumerate(features.lights):
        x, y = _latlon_to_enu(
            light.lat, light.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(
            light.lat, light.lon, terrain, bbox
        )
        models.append(_light_model(f'light_{i:04d}', x, y, z=z))

    if not models:
        return ''

    return '\n\n'.join(models)
