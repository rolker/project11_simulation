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
import random
import re

from osgeo import ogr

from .coordinates import latlon_to_enu

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

# Default building colors by source (ambient, diffuse)
_DEFAULT_S57_BUILDING_COLORS = ('0.6 0.6 0.55 1.0', '0.7 0.7 0.65 1.0')  # warm beige
_DEFAULT_OSM_BUILDING_COLORS = ('0.55 0.58 0.62 1.0', '0.65 0.68 0.72 1.0')  # cool gray-blue

# OSM building:material -> (ambient, diffuse) SDF color strings
_OSM_MATERIAL_COLORS = {
    'brick':    ('0.6 0.3 0.2 1.0', '0.7 0.4 0.3 1.0'),
    'concrete': ('0.6 0.6 0.6 1.0', '0.7 0.7 0.7 1.0'),
    'wood':     ('0.5 0.35 0.2 1.0', '0.6 0.45 0.3 1.0'),
    'metal':    ('0.5 0.5 0.55 1.0', '0.6 0.6 0.65 1.0'),
    'glass':    ('0.4 0.5 0.6 1.0', '0.5 0.6 0.7 1.0'),
    'stone':    ('0.55 0.5 0.45 1.0', '0.65 0.6 0.55 1.0'),
}

# Named CSS colors -> (R, G, B) for building:colour overrides
_CSS_COLORS = {
    'red':     (0.8, 0.2, 0.2),
    'blue':    (0.2, 0.3, 0.8),
    'green':   (0.2, 0.6, 0.2),
    'yellow':  (0.8, 0.8, 0.2),
    'white':   (0.9, 0.9, 0.9),
    'black':   (0.15, 0.15, 0.15),
    'grey':    (0.5, 0.5, 0.5),
    'gray':    (0.5, 0.5, 0.5),
    'brown':   (0.55, 0.35, 0.2),
    'beige':   (0.8, 0.75, 0.65),
    'tan':     (0.7, 0.6, 0.45),
    'orange':  (0.85, 0.5, 0.2),
    'pink':    (0.85, 0.6, 0.65),
    'maroon':  (0.5, 0.15, 0.15),
}


def _parse_hex_color(hex_str: str):
    """Parse a hex color string (#RGB or #RRGGBB) to (R, G, B) floats."""
    s = hex_str.lstrip('#')
    if len(s) == 3:
        r, g, b = int(s[0], 16) / 15.0, int(s[1], 16) / 15.0, int(s[2], 16) / 15.0
    elif len(s) == 6:
        r = int(s[0:2], 16) / 255.0
        g = int(s[2:4], 16) / 255.0
        b = int(s[4:6], 16) / 255.0
    else:
        return None
    return (r, g, b)


def _osm_material_to_colors(material: str, colour: str):
    """Convert OSM building:material and building:colour to SDF colors.

    Returns (ambient, diffuse) as SDF color strings.
    """
    # Colour override takes priority
    if colour:
        colour_lower = colour.lower().strip()
        rgb = _CSS_COLORS.get(colour_lower)
        if rgb is None and colour_lower.startswith('#'):
            rgb = _parse_hex_color(colour_lower)
        if rgb is not None:
            r, g, b = rgb
            amb = f'{r * 0.85:.2f} {g * 0.85:.2f} {b * 0.85:.2f} 1.0'
            dif = f'{r:.2f} {g:.2f} {b:.2f} 1.0'
            return amb, dif

    # Fall back to material
    if material:
        result = _OSM_MATERIAL_COLORS.get(material.lower().strip())
        if result is not None:
            return result

    # Default OSM building colors (callers with S57 data use their own default)
    return _DEFAULT_OSM_BUILDING_COLORS


def _sanitize_objnam(objnam: str) -> str:
    """Sanitize S57 OBJNAM for use as part of a model name.

    Lowercase, replace non-alphanumeric chars with underscores, collapse
    runs of underscores, strip leading/trailing underscores, truncate to
    40 characters.
    """
    name = objnam.lower()
    name = re.sub(r'[^a-z0-9]+', '_', name)
    name = name.strip('_')
    return name[:40]


def _latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """Convert WGS84 lat/lon to local ENU meters via ECEF.

    Thin wrapper around coordinates.latlon_to_enu for backward compatibility.
    """
    return latlon_to_enu(lat, lon, ref_lat, ref_lon)


def _sample_terrain_elevation(east, north, terrain, terrain_bounds):
    """Look up terrain elevation at an ENU position via bilinear interpolation.

    Args:
        east: Easting in meters (ENU).
        north: Northing in meters (ENU).
        terrain: 2D numpy array, north-up (row 0 = north).
        terrain_bounds: dict with 'min_east', 'max_east', 'min_north',
            'max_north' defining the ENU extent of the terrain grid.

    Returns:
        Elevation in meters, or 0.0 if outside bounds.
    """
    if terrain is None:
        return 0.0
    rows, cols = terrain.shape
    min_east = terrain_bounds['min_east']
    max_east = terrain_bounds['max_east']
    min_north = terrain_bounds['min_north']
    max_north = terrain_bounds['max_north']

    # Fractional column (west=0, east=cols-1)
    fc = (east - min_east) / (max_east - min_east) * (cols - 1)
    # Fractional row (north=0, south=rows-1)
    fr = (max_north - north) / (max_north - min_north) * (rows - 1)

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
    collision=True,
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
        + (
            f'        <collision name="collision">\n'
            f'          <geometry>\n'
            f'            <polyline>\n'
            f'              <height>{height:.1f}</height>\n'
            f'{points_str}\n'
            f'            </polyline>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


def _buoy_model(name, x, y, colour_code, collision=True):
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
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 0.4 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.4</radius>\n'
            f'              <length>0.8</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'        <visual name="top">\n'
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


def _beacon_model(name, x, y, z=0.0, colour_code=0, collision=True):
    """Generate an SDF beacon model (tall cylinder with optional color)."""
    if colour_code and colour_code in _S57_COLOURS:
        r, g, b = _S57_COLOURS[colour_code]
        amb = f'{r * 0.8:.1f} {g * 0.8:.1f} {b * 0.8:.1f} 1.0'
        dif = f'{r:.1f} {g:.1f} {b:.1f} 1.0'
    else:
        amb = '0.5 0.5 0.5 1.0'
        dif = '0.6 0.6 0.6 1.0'
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
        f'            <ambient>{amb}</ambient>\n'
        f'            <diffuse>{dif}</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 1.5 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.15</radius>\n'
            f'              <length>3.0</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


_LIGHT_DEFAULT_HEIGHT = 5.0

# Map S57 COLOUR codes to lamp (ambient, diffuse) colors
_LIGHT_COLOURS = {
    1: ('1.0 1.0 1.0 1.0', '1.0 1.0 1.0 1.0'),    # white
    3: ('1.0 0.2 0.2 1.0', '1.0 0.3 0.3 1.0'),    # red
    4: ('0.2 1.0 0.2 1.0', '0.3 1.0 0.3 1.0'),    # green
    6: ('1.0 1.0 0.0 1.0', '1.0 1.0 0.0 1.0'),    # yellow
}
_LIGHT_DEFAULT_COLOUR = ('1.0 1.0 0.0 1.0', '1.0 1.0 0.0 1.0')  # yellow


def _light_model(name, x, y, z=0.0, tower_height=0.0, colour_code=0,
                 collision=True):
    """Generate an SDF light structure model (pole + colored sphere on top)."""
    height = tower_height if tower_height > 0 else _LIGHT_DEFAULT_HEIGHT
    pole_center = height / 2.0
    lamp_z = height + 0.15
    lamp_amb, lamp_dif = _LIGHT_COLOURS.get(colour_code, _LIGHT_DEFAULT_COLOUR)
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="pole">\n'
        f'          <pose>0 0 {pole_center:.2f} 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.1</radius>\n'
        f'              <length>{height:.1f}</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.3 0.3 0.3 1.0</ambient>\n'
        f'            <diffuse>0.4 0.4 0.4 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 {pole_center:.2f} 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.1</radius>\n'
            f'              <length>{height:.1f}</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'        <visual name="lamp">\n'
        f'          <pose>0 0 {lamp_z:.2f} 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <sphere>\n'
        f'              <radius>0.15</radius>\n'
        f'            </sphere>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>{lamp_amb}</ambient>\n'
        f'            <diffuse>{lamp_dif}</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


def _coastline_wall_model(name, geometry, center_lat, center_lon, height=5.0,
                          wall_width=0.3):
    """Generate SDF wall segments along a coastline linestring for debugging.

    Each segment between consecutive points becomes a thin box oriented
    along the segment direction.
    """
    geom_type = geometry.GetGeometryType() & 0xFF
    if geom_type == 2:  # wkbLineString
        lines = [geometry]
    elif geom_type == 5:  # wkbMultiLineString
        lines = [geometry.GetGeometryRef(i)
                 for i in range(geometry.GetGeometryCount())]
    else:
        return ''

    segments = []
    seg_idx = 0
    for line in lines:
        n = line.GetPointCount()
        for i in range(n - 1):
            lon0, lat0 = line.GetX(i), line.GetY(i)
            lon1, lat1 = line.GetX(i + 1), line.GetY(i + 1)
            x0, y0 = _latlon_to_enu(lat0, lon0, center_lat, center_lon)
            x1, y1 = _latlon_to_enu(lat1, lon1, center_lat, center_lon)
            mx = (x0 + x1) / 2.0
            my = (y0 + y1) / 2.0
            dx, dy = x1 - x0, y1 - y0
            length = math.sqrt(dx * dx + dy * dy)
            if length < 0.1:
                continue
            yaw = math.atan2(dy, dx)
            segments.append(
                f'    <model name="{name}_seg{seg_idx:04d}">\n'
                f'      <static>true</static>\n'
                f'      <pose>{mx:.2f} {my:.2f} {height / 2:.2f} '
                f'0 0 {yaw:.4f}</pose>\n'
                f'      <link name="link">\n'
                f'        <visual name="visual">\n'
                f'          <geometry>\n'
                f'            <box>\n'
                f'              <size>{length:.2f} {wall_width} '
                f'{height:.1f}</size>\n'
                f'            </box>\n'
                f'          </geometry>\n'
                f'          <material>\n'
                f'            <ambient>0.0 0.9 0.9 1.0</ambient>\n'
                f'            <diffuse>0.0 1.0 1.0 1.0</diffuse>\n'
                f'          </material>\n'
                f'        </visual>\n'
                f'      </link>\n'
                f'    </model>'
            )
            seg_idx += 1

    return '\n\n'.join(segments)


# SLCONS category-dependent rendering parameters
_SLCONS_PARAMS = {
    # catslc: (height, wall_width, ambient, diffuse)
    1:  (3.0, 1.0, '0.6 0.6 0.6 1.0', '0.7 0.7 0.7 1.0'),      # breakwater — grey
    2:  (1.5, 1.0, '0.2 0.5 0.2 1.0', '0.3 0.6 0.3 1.0'),      # groyne — green
    4:  (2.0, 1.0, '0.6 0.45 0.2 1.0', '0.7 0.55 0.3 1.0'),    # pier/jetty — brown
    6:  (2.5, 1.0, '0.2 0.3 0.7 1.0', '0.3 0.4 0.8 1.0'),      # wharf/quay — blue
    8:  (1.0, 0.5, '0.5 0.5 0.45 1.0', '0.6 0.6 0.55 1.0'),    # rip rap — tan
    9:  (1.5, 0.5, '0.7 0.7 0.2 1.0', '0.8 0.8 0.3 1.0'),      # landing steps — yellow
    10: (3.0, 1.0, '0.7 0.2 0.2 1.0', '0.8 0.3 0.3 1.0'),      # sea wall — red
    12: (1.0, 1.0, '0.5 0.2 0.6 1.0', '0.6 0.3 0.7 1.0'),      # ramp — purple
    13: (1.0, 1.0, '0.2 0.6 0.6 1.0', '0.3 0.7 0.7 1.0'),      # slipway — cyan
    14: (1.5, 0.5, '0.7 0.5 0.2 1.0', '0.8 0.6 0.3 1.0'),      # fender — orange
}
_SLCONS_DEFAULT = (2.0, 1.0, '0.55 0.55 0.50 1.0', '0.65 0.65 0.60 1.0')

# Map CATSLC codes to skip-category names so individual SLCONS types
# can be excluded via --skip-categories (e.g. "rip_rap,groynes").
_CATSLC_SKIP_NAME = {
    1: 'breakwaters',
    2: 'groynes',
    4: 'piers',
    6: 'wharves',
    8: 'rip_rap',
    9: 'landing_steps',
    10: 'sea_walls',
    12: 'ramps',
    13: 'slipways',
    14: 'fenders',
}


def _simplify_enu_linestring(enu_pts, min_seg_len, min_angle_deg=5.0):
    """Remove nearly-collinear vertices and segments shorter than min_seg_len.

    This prevents the offset polygon from self-intersecting at very
    short segments or near-180° bends.
    """
    if len(enu_pts) <= 2:
        return list(enu_pts)

    min_cos = math.cos(math.radians(min_angle_deg))
    result = [enu_pts[0]]

    for i in range(1, len(enu_pts) - 1):
        px, py = result[-1]
        cx, cy = enu_pts[i]
        nx, ny = enu_pts[i + 1]

        dx0, dy0 = cx - px, cy - py
        dx1, dy1 = nx - cx, ny - cy
        l0 = math.sqrt(dx0 * dx0 + dy0 * dy0)
        l1 = math.sqrt(dx1 * dx1 + dy1 * dy1)

        # Skip point if incoming segment is too short
        if l0 < min_seg_len:
            continue

        # Skip nearly collinear points (angle between segments < threshold)
        if l0 > 1e-9 and l1 > 1e-9:
            dot = (dx0 * dx1 + dy0 * dy1) / (l0 * l1)
            if dot > min_cos:
                continue

        result.append(enu_pts[i])

    result.append(enu_pts[-1])

    # Remove final short trailing segment
    if len(result) > 2:
        dx = result[-1][0] - result[-2][0]
        dy = result[-1][1] - result[-2][1]
        if math.sqrt(dx * dx + dy * dy) < min_seg_len:
            result[-2] = result[-1]
            result.pop()

    return result


def _offset_open(pts, hw, max_miter):
    """Offset an open linestring into a polygon (right fwd + left back)."""
    n = len(pts)
    if n < 2:
        return []
    normals = []
    for i in range(n - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        sl = math.sqrt(dx * dx + dy * dy)
        if sl < 1e-9:
            normals.append((0.0, 0.0))
        else:
            normals.append((dy / sl, -dx / sl))

    def _miter(pt, nb, na):
        mx = nb[0] + na[0]
        my = nb[1] + na[1]
        mlen = math.sqrt(mx * mx + my * my)
        if mlen < 1e-9:
            mx, my = nb
            scale = hw
        else:
            mx /= mlen
            my /= mlen
            dot = mx * nb[0] + my * nb[1]
            scale = min(hw / dot, max_miter) if abs(dot) > 0.01 else max_miter
        return ((pt[0] + mx * scale, pt[1] + my * scale),
                (pt[0] - mx * scale, pt[1] - my * scale))

    right, left = [], []
    for i in range(n):
        if i == 0:
            nx, ny = normals[0]
            right.append((pts[0][0] + nx * hw, pts[0][1] + ny * hw))
            left.append((pts[0][0] - nx * hw, pts[0][1] - ny * hw))
        elif i == n - 1:
            nx, ny = normals[-1]
            right.append((pts[i][0] + nx * hw, pts[i][1] + ny * hw))
            left.append((pts[i][0] - nx * hw, pts[i][1] - ny * hw))
        else:
            r, l = _miter(pts[i], normals[i - 1], normals[i])
            right.append(r)
            left.append(l)
    return right + list(reversed(left))


def _offset_linestring_to_polygon(enu_pts, half_width):
    """Offset a linestring by half_width on each side to create a polygon.

    Uses miter joins at internal vertices for square corners.  At very
    acute angles (< 30°) the miter is clamped to 3× half_width to avoid
    extreme spikes.  Input is simplified first to remove segments shorter
    than the wall width that would cause self-intersection.

    Returns list of (x, y) tuples forming a closed polygon (right side
    forward, then left side backward, forming a loop).
    """
    # Simplify to avoid self-intersection from short segments
    enu_pts = _simplify_enu_linestring(enu_pts, half_width * 2.0)

    # Detect closed linestring (first == last point)
    is_closed = (len(enu_pts) >= 4
                 and abs(enu_pts[0][0] - enu_pts[-1][0]) < 0.01
                 and abs(enu_pts[0][1] - enu_pts[-1][1]) < 0.01)
    if is_closed:
        enu_pts = enu_pts[:-1]  # remove duplicate closing point

    n = len(enu_pts)
    if n < 2:
        return []

    hw = half_width
    max_miter = 3.0 * hw  # clamp for very acute angles

    # Compute unit normals (perpendicular, pointing right) per segment
    n_segs = n if is_closed else n - 1
    normals = []
    for i in range(n_segs):
        j = (i + 1) % n
        dx = enu_pts[j][0] - enu_pts[i][0]
        dy = enu_pts[j][1] - enu_pts[i][1]
        seg_len = math.sqrt(dx * dx + dy * dy)
        if seg_len < 1e-9:
            normals.append((0.0, 0.0))
        else:
            # Right-hand normal: rotate direction 90° clockwise
            normals.append((dy / seg_len, -dx / seg_len))

    def _miter_offset(pt, norm_before, norm_after):
        """Compute right and left offset points at a mitered vertex."""
        mx = norm_before[0] + norm_after[0]
        my = norm_before[1] + norm_after[1]
        mlen = math.sqrt(mx * mx + my * my)
        if mlen < 1e-9:
            mx, my = norm_before
            scale = hw
        else:
            mx /= mlen
            my /= mlen
            dot = mx * norm_before[0] + my * norm_before[1]
            if abs(dot) < 0.01:
                scale = max_miter
            else:
                scale = min(hw / dot, max_miter)
        return ((pt[0] + mx * scale, pt[1] + my * scale),
                (pt[0] - mx * scale, pt[1] - my * scale))

    # Compute offset points at each vertex using miter joins
    right_pts = []
    left_pts = []

    for i in range(n):
        if is_closed:
            # All vertices are internal — wrap around
            nb = normals[(i - 1) % n_segs]
            na = normals[i % n_segs]
            r, l = _miter_offset(enu_pts[i], nb, na)
        elif i == 0:
            # Start cap: use first segment's normal
            nx, ny = normals[0]
            r = (enu_pts[i][0] + nx * hw, enu_pts[i][1] + ny * hw)
            l = (enu_pts[i][0] - nx * hw, enu_pts[i][1] - ny * hw)
        elif i == n - 1:
            # End cap: use last segment's normal
            nx, ny = normals[-1]
            r = (enu_pts[i][0] + nx * hw, enu_pts[i][1] + ny * hw)
            l = (enu_pts[i][0] - nx * hw, enu_pts[i][1] - ny * hw)
        else:
            # Internal vertex: miter join
            r, l = _miter_offset(enu_pts[i], normals[i - 1], normals[i])
        right_pts.append(r)
        left_pts.append(l)

    if is_closed:
        # Closed linestring: split at midpoint into two open segments,
        # each offset independently.  This avoids the figure-8 self-
        # intersection that arises from concatenating right+left loops.
        mid = n // 2
        seg1 = enu_pts[:mid + 1]  # first half (overlaps at midpoint)
        seg2 = enu_pts[mid:] + [enu_pts[0]]  # second half, close back
        poly1 = _offset_open(seg1, hw, max_miter)
        poly2 = _offset_open(seg2, hw, max_miter)
        return ('split', poly1, poly2)

    # Open linestring: right side forward, left side backward
    return right_pts + list(reversed(left_pts))


def _slcons_wall_model(name, geometry, center_lat, center_lon,
                        height=2.0, wall_width=1.0, z=0.0,
                        ambient='0.55 0.55 0.50 1.0',
                        diffuse='0.65 0.65 0.60 1.0',
                        collision=True, seg_budget=None):
    """Generate SDF polyline extrusion along a shoreline construction.

    Offsets the linestring by wall_width/2 on each side to create a
    polygon with mitered corners, then extrudes it as a single polyline
    model.  This produces clean square joints at all bend angles.
    """
    geom_type = geometry.GetGeometryType() & 0xFF
    if geom_type == 2:  # wkbLineString
        lines = [geometry]
    elif geom_type == 5:  # wkbMultiLineString
        lines = [geometry.GetGeometryRef(i)
                 for i in range(geometry.GetGeometryCount())]
    elif geom_type == 7:  # wkbGeometryCollection
        lines = []
        for i in range(geometry.GetGeometryCount()):
            sub = geometry.GetGeometryRef(i)
            stype = sub.GetGeometryType() & 0xFF
            if stype in (2, 5):  # LineString or MultiLineString
                lines.append(sub)
    else:
        return '', 0

    models = []
    model_idx = 0
    for line in lines:
        if seg_budget is not None and model_idx >= seg_budget:
            break
        # Handle MultiLineString sub-geometries
        if (line.GetGeometryType() & 0xFF) == 5:
            sub_lines = [line.GetGeometryRef(j)
                         for j in range(line.GetGeometryCount())]
        else:
            sub_lines = [line]
        for sline in sub_lines:
            if seg_budget is not None and model_idx >= seg_budget:
                break
            n = sline.GetPointCount()
            if n < 2:
                continue

            # Convert to ENU
            enu_pts = []
            for pi in range(n):
                lon_p, lat_p = sline.GetX(pi), sline.GetY(pi)
                enu_pts.append(
                    _latlon_to_enu(lat_p, lon_p, center_lat, center_lon))

            # Offset linestring to polygon
            result = _offset_linestring_to_polygon(
                enu_pts, wall_width / 2.0)

            # Handle closed linestrings (split into two open segments)
            if isinstance(result, tuple) and result[0] == 'split':
                _, poly1, poly2 = result
                # Emit each half as a separate model
                for split_pts in (poly1, poly2):
                    if len(split_pts) < 3:
                        continue
                    scx = sum(p[0] for p in split_pts) / len(split_pts)
                    scy = sum(p[1] for p in split_pts) / len(split_pts)
                    pts_lines = []
                    for px, py in split_pts:
                        pts_lines.append(
                            f'          <point>{px - scx:.2f}'
                            f' {py - scy:.2f}</point>')
                    pts_lines.append(
                        f'          <point>{split_pts[0][0] - scx:.2f}'
                        f' {split_pts[0][1] - scy:.2f}</point>')
                    ps = '\n'.join(pts_lines)
                    pl_sdf = (
                        f'            <polyline>\n'
                        f'              <height>{height:.1f}</height>\n'
                        f'{ps}\n'
                        f'            </polyline>')
                    m = (
                        f'    <model name="{name}_p{model_idx:04d}">\n'
                        f'      <static>true</static>\n'
                        f'      <pose>{scx:.2f} {scy:.2f} {z:.2f}'
                        f' 0 0 0</pose>\n'
                        f'      <link name="link">\n'
                        f'        <visual name="visual">\n'
                        f'          <geometry>\n'
                        f'{pl_sdf}\n'
                        f'          </geometry>\n'
                        f'          <material>\n'
                        f'            <ambient>{ambient}</ambient>\n'
                        f'            <diffuse>{diffuse}</diffuse>\n'
                        f'          </material>\n'
                        f'        </visual>\n'
                        + (
                            f'        <collision name="collision">\n'
                            f'          <geometry>\n'
                            f'{pl_sdf}\n'
                            f'          </geometry>\n'
                            f'        </collision>\n'
                            if collision else ''
                        )
                        + f'      </link>\n'
                        f'    </model>'
                    )
                    models.append(m)
                    model_idx += 1
                continue

            poly_pts = result
            if len(poly_pts) < 3:
                continue
            cx = sum(p[0] for p in poly_pts) / len(poly_pts)
            cy = sum(p[1] for p in poly_pts) / len(poly_pts)

            points = []
            for px, py in poly_pts:
                points.append(
                    f'          <point>{px - cx:.2f}'
                    f' {py - cy:.2f}</point>')
            points.append(
                f'          <point>{poly_pts[0][0] - cx:.2f}'
                f' {poly_pts[0][1] - cy:.2f}</point>')
            polyline_sdf = (
                f'            <polyline>\n'
                f'              <height>{height:.1f}</height>\n'
                + '\n'.join(points) + '\n'
                f'            </polyline>')

            model = (
                f'    <model name="{name}_p{model_idx:04d}">\n'
                f'      <static>true</static>\n'
                f'      <pose>{cx:.2f} {cy:.2f} {z:.2f} 0 0 0</pose>\n'
                f'      <link name="link">\n'
                f'        <visual name="visual">\n'
                f'          <geometry>\n'
                f'{polyline_sdf}\n'
                f'          </geometry>\n'
                f'          <material>\n'
                f'            <ambient>{ambient}</ambient>\n'
                f'            <diffuse>{diffuse}</diffuse>\n'
                f'          </material>\n'
                f'        </visual>\n'
                + (
                    f'        <collision name="collision">\n'
                    f'          <geometry>\n'
                    f'{polyline_sdf}\n'
                    f'          </geometry>\n'
                    f'        </collision>\n'
                    if collision else ''
                )
                + f'      </link>\n'
                f'    </model>'
            )
            models.append(model)
            model_idx += 1

    return '\n\n'.join(models), model_idx


def _slcons_point_model(name, x, y, z=0.0, collision=True):
    """Generate an SDF model for a point SLCONS feature (small bollard)."""
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <pose>0 0 0.5 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>0.3</radius>\n'
        f'              <length>1.0</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.55 0.55 0.50 1.0</ambient>\n'
        f'            <diffuse>0.65 0.65 0.60 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 0.5 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.3</radius>\n'
            f'              <length>1.0</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


def _pile_model(name, x, y, collision=True):
    """Generate an SDF pile model (thin cylinder at water level)."""
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\n'
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
        f'            <ambient>0.45 0.40 0.35 1.0</ambient>\n'
        f'            <diffuse>0.55 0.50 0.45 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 1.5 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.15</radius>\n'
            f'              <length>3.0</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


def _morfac_model(name, x, y, z, catmor, collision=True):
    """Generate an SDF mooring facility model based on category."""
    if catmor == 7:  # mooring buoy
        return _buoy_model(name, x, y, colour_code=6, collision=collision)
    elif catmor in (1, 2):  # dolphin
        return (
            f'    <model name="{name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="visual">\n'
            f'          <pose>0 0 1.0 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.3</radius>\n'
            f'              <length>2.0</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'          <material>\n'
            f'            <ambient>0.50 0.40 0.30 1.0</ambient>\n'
            f'            <diffuse>0.60 0.50 0.40 1.0</diffuse>\n'
            f'          </material>\n'
            f'        </visual>\n'
            + (
                f'        <collision name="collision">\n'
                f'          <pose>0 0 1.0 0 0 0</pose>\n'
                f'          <geometry>\n'
                f'            <cylinder>\n'
                f'              <radius>0.3</radius>\n'
                f'              <length>2.0</length>\n'
                f'            </cylinder>\n'
                f'          </geometry>\n'
                f'        </collision>\n'
                if collision else ''
            )
            + f'      </link>\n'
            f'    </model>'
        )
    elif catmor == 3:  # bollard
        return (
            f'    <model name="{name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="visual">\n'
            f'          <pose>0 0 0.25 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.2</radius>\n'
            f'              <length>0.5</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'          <material>\n'
            f'            <ambient>0.3 0.3 0.3 1.0</ambient>\n'
            f'            <diffuse>0.4 0.4 0.4 1.0</diffuse>\n'
            f'          </material>\n'
            f'        </visual>\n'
            + (
                f'        <collision name="collision">\n'
                f'          <pose>0 0 0.25 0 0 0</pose>\n'
                f'          <geometry>\n'
                f'            <cylinder>\n'
                f'              <radius>0.2</radius>\n'
                f'              <length>0.5</length>\n'
                f'            </cylinder>\n'
                f'          </geometry>\n'
                f'        </collision>\n'
                if collision else ''
            )
            + f'      </link>\n'
            f'    </model>'
        )
    elif catmor == 5:  # post
        return _pile_model(name, x, y, collision=collision)
    else:  # generic
        return (
            f'    <model name="{name}">\n'
            f'      <static>true</static>\n'
            f'      <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>\n'
            f'      <link name="link">\n'
            f'        <visual name="visual">\n'
            f'          <pose>0 0 1.0 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>0.25</radius>\n'
            f'              <length>2.0</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'          <material>\n'
            f'            <ambient>0.45 0.40 0.35 1.0</ambient>\n'
            f'            <diffuse>0.55 0.50 0.45 1.0</diffuse>\n'
            f'          </material>\n'
            f'        </visual>\n'
            + (
                f'        <collision name="collision">\n'
                f'          <pose>0 0 1.0 0 0 0</pose>\n'
                f'          <geometry>\n'
                f'            <cylinder>\n'
                f'              <radius>0.25</radius>\n'
                f'              <length>2.0</length>\n'
                f'            </cylinder>\n'
                f'          </geometry>\n'
                f'        </collision>\n'
                if collision else ''
            )
            + f'      </link>\n'
            f'    </model>'
        )


def _crane_model(name, x, y, z, catcrn, height, collision=True):
    """Generate an SDF crane model (tall box, industrial yellow)."""
    h = height if height > 0 else 20.0
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <pose>0 0 {h / 2:.2f} 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <box>\n'
        f'              <size>3.0 3.0 {h:.1f}</size>\n'
        f'            </box>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.8 0.7 0.1 1.0</ambient>\n'
        f'            <diffuse>0.9 0.8 0.2 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 {h / 2:.2f} 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <box>\n'
            f'              <size>3.0 3.0 {h:.1f}</size>\n'
            f'            </box>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


def _pylon_model(name, x, y, z, height, collision=True):
    """Generate an SDF pylon model (thick gray cylinder)."""
    h = height if height > 0 else 15.0
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <pose>0 0 {h / 2:.2f} 0 0 0</pose>\n'
        f'          <geometry>\n'
        f'            <cylinder>\n'
        f'              <radius>1.0</radius>\n'
        f'              <length>{h:.1f}</length>\n'
        f'            </cylinder>\n'
        f'          </geometry>\n'
        f'          <material>\n'
        f'            <ambient>0.55 0.55 0.55 1.0</ambient>\n'
        f'            <diffuse>0.65 0.65 0.65 1.0</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        + (
            f'        <collision name="collision">\n'
            f'          <pose>0 0 {h / 2:.2f} 0 0 0</pose>\n'
            f'          <geometry>\n'
            f'            <cylinder>\n'
            f'              <radius>1.0</radius>\n'
            f'              <length>{h:.1f}</length>\n'
            f'            </cylinder>\n'
            f'          </geometry>\n'
            f'        </collision>\n'
            if collision else ''
        )
        + f'      </link>\n'
        f'    </model>'
    )


def _wrap_group(group_name, model_lists):
    """Wrap non-empty model lists into a nested container model.

    Args:
        group_name: Name for the top-level container (e.g. 's57_features').
        model_lists: dict mapping type name to list of model XML strings.

    Returns:
        SDF string for the container model, or empty string if all empty.
    """
    type_groups = []
    for type_name, models in model_lists.items():
        if not models:
            continue
        inner = '\n'.join(f'  {line}' for m in models for line in m.split('\n'))
        type_groups.append(
            f'      <model name="{type_name}">\n'
            f'{inner}\n'
            f'      </model>'
        )
    if not type_groups:
        return ''
    inner_xml = '\n'.join(type_groups)
    return (
        f'    <model name="{group_name}">\n'
        f'      <static>true</static>\n'
        f'{inner_xml}\n'
        f'    </model>'
    )


# Tree generation constants
_TREE_SPACING_DEFAULT = 8.0  # meters between grid points at natural density
_TREE_MAX_COUNT = 500        # default cap on total tree count
_TRUNK_RADIUS = 0.15
_TRUNK_HEIGHT = 3.0
_CANOPY_RADIUS = 2.5
_CANOPY_HEIGHT = 5.0
_TREE_HEIGHT_VARIANCE = 0.3  # +/- fraction of nominal height


def _compute_tree_spacing(natural_list, center_lat, center_lon, max_trees):
    """Compute grid spacing so total trees stays under max_trees.

    Estimates total wood area in m² using ENU-projected bounding boxes,
    then widens spacing if the default would exceed the cap.
    """
    total_area = 0.0
    for nat in natural_list:
        if nat.natural != 'wood':
            continue
        geom = nat.geometry
        if geom is None or geom.IsEmpty():
            continue
        # Use bounding box area as upper estimate (simple + reliable)
        env = geom.GetEnvelope()  # (minX, maxX, minY, maxY)
        sw = _latlon_to_enu(env[2], env[0], center_lat, center_lon)
        ne = _latlon_to_enu(env[3], env[1], center_lat, center_lon)
        bbox_area = abs(ne[0] - sw[0]) * abs(ne[1] - sw[1])
        # Polygons typically fill ~50-70% of their bbox
        total_area += bbox_area * 0.5

    if total_area == 0:
        return _TREE_SPACING_DEFAULT

    # At default spacing, roughly one tree per spacing^2 of polygon area
    trees_at_default = total_area / (_TREE_SPACING_DEFAULT ** 2)
    if trees_at_default <= max_trees:
        return _TREE_SPACING_DEFAULT

    # Widen spacing: trees ~ area / spacing^2
    spacing = math.sqrt(total_area / max_trees)
    return spacing


def _scatter_trees(natural_list, center_lat, center_lon, terrain,
                   terrain_bounds, max_trees=_TREE_MAX_COUNT):
    """Generate SDF tree models scattered within natural=wood polygons.

    Uses a grid-based approach with jitter for natural-looking placement.
    Spacing is adjusted so total count stays under max_trees.
    Each tree is a cylinder trunk + cone canopy using SDF primitives.
    """
    spacing = _compute_tree_spacing(
        natural_list, center_lat, center_lon, max_trees)
    jitter = spacing * 0.375  # scale jitter with spacing

    tree_models = []
    tree_idx = 0
    rng = random.Random(42)  # deterministic for reproducible worlds

    for nat in natural_list:
        if nat.natural != 'wood':
            continue

        geom = nat.geometry
        if geom is None or geom.IsEmpty():
            continue

        # Get polygon bounding box in lat/lon
        env = geom.GetEnvelope()  # (minX, maxX, minY, maxY)
        min_lon, max_lon, min_lat, max_lat = env

        # Convert bbox corners to ENU to determine grid extent
        sw = _latlon_to_enu(min_lat, min_lon, center_lat, center_lon)
        ne = _latlon_to_enu(max_lat, max_lon, center_lat, center_lon)

        # Grid over the ENU bounding box
        e = sw[0]
        while e < ne[0] and tree_idx < max_trees:
            n = sw[1]
            while n < ne[1] and tree_idx < max_trees:
                # Add jitter
                je = e + rng.uniform(-jitter, jitter)
                jn = n + rng.uniform(-jitter, jitter)

                # Convert back to lat/lon for point-in-polygon test
                # Approximate inverse: use linear scaling from center
                lat_per_m = 1.0 / 111320.0
                lon_per_m = 1.0 / (111320.0 * math.cos(
                    math.radians(center_lat)))
                pt_lat = center_lat + jn * lat_per_m
                pt_lon = center_lon + je * lon_per_m

                pt = ogr.Geometry(ogr.wkbPoint)
                pt.AddPoint(pt_lon, pt_lat)

                if geom.Contains(pt):
                    z = _sample_terrain_elevation(
                        je, jn, terrain, terrain_bounds)

                    # Randomize tree dimensions
                    scale = 1.0 + rng.uniform(
                        -_TREE_HEIGHT_VARIANCE, _TREE_HEIGHT_VARIANCE)
                    trunk_h = _TRUNK_HEIGHT * scale
                    canopy_h = _CANOPY_HEIGHT * scale
                    canopy_r = _CANOPY_RADIUS * scale
                    trunk_r = _TRUNK_RADIUS * scale

                    # Random yaw for visual variety
                    yaw = rng.uniform(0, 2 * math.pi)

                    tree_models.append(
                        f'    <model name="tree_{tree_idx:05d}">\n'
                        f'      <static>true</static>\n'
                        f'      <pose>{je:.2f} {jn:.2f} {z:.2f}'
                        f' 0 0 {yaw:.3f}</pose>\n'
                        f'      <link name="link">\n'
                        f'        <visual name="trunk">\n'
                        f'          <pose>0 0 {trunk_h / 2.0:.2f}'
                        f' 0 0 0</pose>\n'
                        f'          <geometry>\n'
                        f'            <cylinder>\n'
                        f'              <radius>{trunk_r:.2f}</radius>\n'
                        f'              <length>{trunk_h:.2f}</length>\n'
                        f'            </cylinder>\n'
                        f'          </geometry>\n'
                        f'          <material>\n'
                        f'            <ambient>0.4 0.25 0.1 1</ambient>\n'
                        f'            <diffuse>0.5 0.3 0.15 1</diffuse>\n'
                        f'          </material>\n'
                        f'        </visual>\n'
                        f'        <visual name="canopy">\n'
                        f'          <pose>0 0 '
                        f'{trunk_h + canopy_h / 2.0:.2f}'
                        f' 0 0 0</pose>\n'
                        f'          <geometry>\n'
                        f'            <cone>\n'
                        f'              <radius>{canopy_r:.2f}</radius>\n'
                        f'              <length>{canopy_h:.2f}</length>\n'
                        f'            </cone>\n'
                        f'          </geometry>\n'
                        f'          <material>\n'
                        f'            <ambient>0.15 0.35 0.1 1</ambient>\n'
                        f'            <diffuse>0.2 0.45 0.15 1</diffuse>\n'
                        f'          </material>\n'
                        f'        </visual>\n'
                        f'      </link>\n'
                        f'    </model>'
                    )
                    tree_idx += 1

                n += spacing
            e += spacing

    return tree_models


def generate_feature_models(
    features, center_lat, center_lon,
    terrain=None, terrain_bounds=None, debug=False,
    skip_categories=None, simplify_tolerance=0.0,
    max_wall_segments=None,
    osm_man_made=None,
    osm_bridge_roads=None,
    osm_natural=None,
    max_trees=_TREE_MAX_COUNT,
):
    """Convert S57 features to grouped SDF <model> XML strings.

    Features are organized into two top-level container models:
    ``s57_features`` and ``osm_features``, each containing sub-groups
    by feature type. This creates a 3-level hierarchy in Gazebo's
    Entity Tree: source -> feature type -> individual models.

    Args:
        features: S57Features instance with extracted chart features.
        center_lat: World center latitude (WGS84 degrees).
        center_lon: World center longitude (WGS84 degrees).
        terrain: Optional 2D numpy array of elevation (north-up, meters).
        terrain_bounds: Optional dict with 'min_east', 'max_east',
            'min_north', 'max_north' defining the ENU extent of the
            terrain grid (meters).
        debug: If True, polygon features use thin (0.2m) extrusions with
            distinct colors per type for footprint visualization.
        skip_categories: Optional set of category names to skip (e.g.
            {'buildings', 'slcons', 'buoys'}).
        simplify_tolerance: Geometry simplification tolerance in degrees.
            Applied to polygon and linestring features before model
            generation.  0 disables simplification.

    Returns:
        Dict with 's57' and 'osm' keys, each containing an SDF string
        for the corresponding container model. Values are empty strings
        if no features to generate for that source.
    """
    skip = skip_categories or set()

    # Land features omit collision geometry — they exist for visual context
    # only and don't need physics interaction with vessels.
    _LAND = False

    # Collect models by type for S57 and OSM sources
    s57_buildings = []
    osm_buildings = []

    # Buildings (BUISGL, LNDMRK, SILTNK)
    for i, building in enumerate(
        features.buildings if 'buildings' not in skip else []
    ):
        centroid = building.geometry.Centroid()
        cx, cy = _latlon_to_enu(
            centroid.GetY(), centroid.GetX(), center_lat, center_lon
        )
        z = _sample_terrain_elevation(cx, cy, terrain, terrain_bounds)
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['building']
        else:
            # Height priority: OSM height > OBJL default
            if building.osm_height is not None:
                height = building.osm_height
            else:
                height = _BUILDING_HEIGHTS.get(building.objl, 5.0)
            # Material colors from OSM
            if building.osm_material or building.osm_colour:
                amb, dif = _osm_material_to_colors(
                    building.osm_material, building.osm_colour,
                )
            elif building.osm_only:
                amb, dif = _DEFAULT_OSM_BUILDING_COLORS
            else:
                amb, dif = _DEFAULT_S57_BUILDING_COLORS
        # Include OBJNAM in model name if available
        name = f'building_{i:04d}'
        if building.objnam:
            suffix = _sanitize_objnam(building.objnam)
            if suffix:
                name = f'building_{i:04d}_{suffix}'
        model = _polygon_to_polyline_model(
            name, building.geometry,
            center_lat, center_lon, height, z=z,
            ambient=amb, diffuse=dif, collision=_LAND,
        )
        if model:
            if building.osm_only:
                osm_buildings.append(model)
            else:
                s57_buildings.append(model)

    # Pontoons (water surface, z=0)
    pontoons = []
    for i, pontoon in enumerate(
        features.pontoons if 'pontoons' not in skip else []
    ):
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['pontoon']
        else:
            height = _PONTOON_HEIGHT
            amb = '0.4 0.35 0.3 1.0'
            dif = '0.5 0.45 0.4 1.0'
        geom_type = pontoon.geometry.GetGeometryType() & 0xFF
        if geom_type in (3, 6):  # Polygon, MultiPolygon
            model = _polygon_to_polyline_model(
                f'pontoon_{i:04d}', pontoon.geometry,
                center_lat, center_lon, height, z=0.0,
                ambient=amb, diffuse=dif,
            )
            if model:
                pontoons.append(model)
        elif geom_type in (2, 5, 7):  # LineString, Multi, Collection
            wall, _ = _slcons_wall_model(
                f'pontoon_{i:04d}', pontoon.geometry,
                center_lat, center_lon,
                height=height, wall_width=1.0,
                ambient=amb, diffuse=dif,
            )
            if wall:
                pontoons.append(wall)

    # Bridges (deck at clearance height, not extruded from 0)
    bridges = []
    for i, bridge in enumerate(
        features.bridges if 'bridges' not in skip else []
    ):
        clearance = (bridge.clearance if bridge.clearance > 0
                     else _BRIDGE_DEFAULT_HEIGHT)
        if debug:
            height = 0.2
            amb, dif = _DEBUG_COLORS['bridge']
        else:
            height = 1.0  # deck thickness
            amb = '0.5 0.5 0.5 1.0'
            dif = '0.6 0.6 0.6 1.0'
        geom_type = bridge.geometry.GetGeometryType() & 0xFF
        if geom_type in (3, 6):  # Polygon, MultiPolygon
            model = _polygon_to_polyline_model(
                f'bridge_{i:04d}', bridge.geometry,
                center_lat, center_lon, height, z=clearance,
                ambient=amb, diffuse=dif,
            )
            if model:
                bridges.append(model)
        elif geom_type in (2, 5, 7):  # LineString, Multi, Collection
            wall, _ = _slcons_wall_model(
                f'bridge_{i:04d}', bridge.geometry,
                center_lat, center_lon,
                height=height, wall_width=4.0, z=clearance,
                ambient=amb, diffuse=dif,
            )
            if wall:
                bridges.append(wall)

    # Buoys (water surface, z=0)
    buoys = []
    for i, buoy in enumerate(
        features.buoys if 'buoys' not in skip else []
    ):
        x, y = _latlon_to_enu(buoy.lat, buoy.lon, center_lat, center_lon)
        buoys.append(_buoy_model(f'buoy_{i:04d}', x, y, buoy.colour))

    # Beacons (on terrain)
    beacons = []
    for i, beacon in enumerate(
        features.beacons if 'beacons' not in skip else []
    ):
        x, y = _latlon_to_enu(
            beacon.lat, beacon.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(x, y, terrain, terrain_bounds)
        beacons.append(_beacon_model(
            f'beacon_{i:04d}', x, y, z=z, colour_code=beacon.colour,
            collision=_LAND,
        ))

    # Lights (on terrain)
    lights = []
    for i, light in enumerate(
        features.lights if 'lights' not in skip else []
    ):
        x, y = _latlon_to_enu(
            light.lat, light.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(x, y, terrain, terrain_bounds)
        lights.append(_light_model(f'light_{i:04d}', x, y, z=z,
                                   tower_height=light.height,
                                   colour_code=light.colour,
                                   collision=_LAND))

    # Shore constructions (SLCONS): line, polygon, and point geometry
    shore_constructions = []
    wall_seg_remaining = max_wall_segments
    for i, sc in enumerate(
        features.shore_constructions if 'shore_constructions' not in skip
        else []
    ):
        if wall_seg_remaining is not None and wall_seg_remaining <= 0:
            break
        # Allow skipping individual SLCONS sub-types (e.g. "rip_rap")
        catslc_skip = _CATSLC_SKIP_NAME.get(sc.catslc)
        if catslc_skip and catslc_skip in skip:
            continue
        params = _SLCONS_PARAMS.get(sc.catslc, _SLCONS_DEFAULT)
        height, wall_width, amb, dif = params

        # Adjust rendering based on water level attribute (all SLCONS types)
        # watlev: 2=always dry, 3=submerged, 4=covers/uncovers (tidal),
        #         7=floating
        if sc.watlev == 7:
            # Floating — slab straddling water surface
            height = 1.0
            z_override = -0.5
        elif sc.watlev == 4:
            # Tidal / covers and uncovers — slab at water surface
            height = 1.0
            z_override = -0.5
        elif sc.watlev == 3:
            # Submerged — thin slab below water
            height = 0.5
            z_override = -1.0
        else:
            z_override = None

        geom_type = sc.geometry.GetGeometryType() & 0xFF
        if geom_type in (2, 5, 7):  # LineString, MultiLineString, Collection
            z = z_override if z_override is not None else 0.0
            wall, n_segs = _slcons_wall_model(
                f'slcons_{i:04d}', sc.geometry,
                center_lat, center_lon,
                height=height, wall_width=wall_width, z=z,
                ambient=amb, diffuse=dif, collision=_LAND,
                seg_budget=wall_seg_remaining,
            )
            if wall:
                shore_constructions.append(wall)
            if wall_seg_remaining is not None:
                wall_seg_remaining -= n_segs
        elif geom_type in (3, 6):  # Polygon, MultiPolygon
            if z_override is not None:
                z = z_override
            else:
                centroid = sc.geometry.Centroid()
                cx, cy = _latlon_to_enu(
                    centroid.GetY(), centroid.GetX(), center_lat, center_lon)
                z = _sample_terrain_elevation(cx, cy, terrain, terrain_bounds)
            model = _polygon_to_polyline_model(
                f'slcons_{i:04d}', sc.geometry,
                center_lat, center_lon, height, z=z,
                ambient=amb, diffuse=dif, collision=_LAND,
            )
            if model:
                shore_constructions.append(model)
        elif geom_type == 1:  # Point
            x, y = _latlon_to_enu(
                sc.geometry.GetY(), sc.geometry.GetX(),
                center_lat, center_lon,
            )
            shore_constructions.append(_slcons_point_model(
                f'slcons_{i:04d}', x, y, collision=_LAND))

    # Piles (at water level)
    piles = []
    for i, pile in enumerate(
        features.piles if 'piles' not in skip else []
    ):
        x, y = _latlon_to_enu(pile.lat, pile.lon, center_lat, center_lon)
        piles.append(_pile_model(f'pile_{i:04d}', x, y))

    # Mooring facilities
    mooring_facilities = []
    for i, mf in enumerate(
        features.mooring_facilities if 'mooring_facilities' not in skip
        else []
    ):
        x, y = _latlon_to_enu(mf.lat, mf.lon, center_lat, center_lon)
        z = _sample_terrain_elevation(x, y, terrain, terrain_bounds)
        mooring_facilities.append(_morfac_model(
            f'morfac_{i:04d}', x, y, z, mf.catmor, collision=_LAND))

    # Cranes (on terrain)
    cranes = []
    for i, crane in enumerate(
        features.cranes if 'cranes' not in skip else []
    ):
        x, y = _latlon_to_enu(
            crane.lat, crane.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(x, y, terrain, terrain_bounds)
        cranes.append(_crane_model(
            f'crane_{i:04d}', x, y, z, crane.catcrn, crane.height,
            collision=_LAND,
        ))

    # Pylons (on terrain)
    pylons = []
    for i, pylon in enumerate(
        features.pylons if 'pylons' not in skip else []
    ):
        x, y = _latlon_to_enu(
            pylon.lat, pylon.lon, center_lat, center_lon
        )
        z = _sample_terrain_elevation(x, y, terrain, terrain_bounds)
        pylons.append(_pylon_model(
            f'pylon_{i:04d}', x, y, z, pylon.height,
            collision=_LAND,
        ))

    # Debug: coastline walls (simplified to reduce segment count)
    coastlines = []
    if debug and features.coastlines:
        for i, coastline in enumerate(features.coastlines):
            tol = simplify_tolerance if simplify_tolerance > 0 else 0.0002
            simplified = coastline.Simplify(tol)
            wall = _coastline_wall_model(
                f'coastline_{i:04d}', simplified,
                center_lat, center_lon,
            )
            if wall:
                coastlines.append(wall)

    # Marine infrastructure from OSM (piers, quays, breakwaters, groynes)
    _MARINE_COLORS = {
        'pier': ('0.65 0.62 0.58 1.0', '0.75 0.72 0.68 1.0'),
        'quay': ('0.55 0.55 0.53 1.0', '0.65 0.65 0.63 1.0'),
        'breakwater': ('0.45 0.45 0.43 1.0', '0.55 0.55 0.53 1.0'),
        'groyne': ('0.45 0.45 0.43 1.0', '0.55 0.55 0.53 1.0'),
    }
    _MARINE_DEFAULT_COLOR = ('0.55 0.55 0.53 1.0', '0.65 0.65 0.63 1.0')
    # Floating pier dimensions: deck freeboard and total slab thickness
    _FLOATING_FREEBOARD = 0.5   # deck height above water (meters)
    _FLOATING_DRAFT = 0.5       # depth below water (meters)
    _FLOATING_THICKNESS = _FLOATING_FREEBOARD + _FLOATING_DRAFT  # total slab
    marine_models = []
    for i, mm in enumerate(osm_man_made or []):
        amb, dif = _MARINE_COLORS.get(mm.man_made, _MARINE_DEFAULT_COLOR)
        is_floating = mm.floating is True
        geom_type = mm.geometry.GetGeometryType() & 0xFF
        if geom_type in (3, 6):  # Polygon — flat extruded slab
            if is_floating:
                z = -_FLOATING_DRAFT
                slab_height = _FLOATING_THICKNESS
            else:
                centroid = mm.geometry.Centroid()
                cx, cy = _latlon_to_enu(
                    centroid.GetY(), centroid.GetX(), center_lat, center_lon)
                z = _sample_terrain_elevation(cx, cy, terrain, terrain_bounds)
                slab_height = 1.0
            model = _polygon_to_polyline_model(
                f'marine_{i:04d}_{mm.man_made}', mm.geometry,
                center_lat, center_lon, slab_height, z=z,
                ambient=amb, diffuse=dif, collision=False,
            )
            if model:
                marine_models.append(model)
        elif geom_type in (2, 5, 7):  # LineString — wall segments
            if is_floating:
                wall_width = 3.0
                height = _FLOATING_THICKNESS
                z = -_FLOATING_DRAFT
            else:
                wall_width = 3.0 if mm.man_made in ('breakwater', 'groyne') else 2.0
                height = 2.0 if mm.man_made in ('breakwater', 'groyne') else 1.0
                z = 0.0
            wall, _ = _slcons_wall_model(
                f'marine_{i:04d}_{mm.man_made}', mm.geometry,
                center_lat, center_lon,
                height=height, wall_width=wall_width, z=z,
                ambient=amb, diffuse=dif, collision=False,
            )
            if wall:
                marine_models.append(wall)

    # OSM bridge roads (gangways, elevated walkways)
    # Gangway: sloped ramp from fixed pier (higher) to floating pier (lower).
    # We detect which end connects to a floating pier to determine slope.
    _GANGWAY_FIXED_Z = 1.5   # fixed end height above water (meters)
    _GANGWAY_FLOAT_Z = 0.5   # floating end height (matches freeboard)
    _GANGWAY_THICKNESS = 0.3
    osm_bridge_models = []
    for i, road in enumerate(osm_bridge_roads or []):
        geom = road.geometry
        n = geom.GetPointCount()
        if n < 2:
            continue

        # Convert endpoints to ENU
        start_enu = _latlon_to_enu(geom.GetY(0), geom.GetX(0),
                                   center_lat, center_lon)
        end_enu = _latlon_to_enu(geom.GetY(n - 1), geom.GetX(n - 1),
                                 center_lat, center_lon)

        # Determine which end is higher (fixed pier) vs lower (floating pier)
        # by checking proximity to floating vs fixed man_made features.
        start_near_float = False
        end_near_float = False
        for mm in (osm_man_made or []):
            if mm.floating is not True:
                continue
            mm_geom = mm.geometry
            # Check distance from gangway endpoints to floating feature.
            # Polygon geometries return 0 for GetPointCount() — iterate
            # the exterior ring instead.
            geom_type = mm_geom.GetGeometryType() & 0xFF
            if geom_type == 3:  # Polygon
                point_source = mm_geom.GetGeometryRef(0)
                if point_source is None:
                    continue
            else:
                point_source = mm_geom
            for pt_idx in range(point_source.GetPointCount()):
                fx, fy = _latlon_to_enu(point_source.GetY(pt_idx),
                                        point_source.GetX(pt_idx),
                                        center_lat, center_lon)
                ds = math.sqrt((start_enu[0] - fx) ** 2
                               + (start_enu[1] - fy) ** 2)
                de = math.sqrt((end_enu[0] - fx) ** 2
                               + (end_enu[1] - fy) ** 2)
                if ds < 5.0:
                    start_near_float = True
                if de < 5.0:
                    end_near_float = True

        if start_near_float and not end_near_float:
            z_start = _GANGWAY_FLOAT_Z
            z_end = _GANGWAY_FIXED_Z
        elif end_near_float and not start_near_float:
            z_start = _GANGWAY_FIXED_Z
            z_end = _GANGWAY_FLOAT_Z
        else:
            # Can't determine — use fixed height
            z_start = _GANGWAY_FIXED_Z
            z_end = _GANGWAY_FIXED_Z

        # Build as individual box segments with linearly interpolated z
        width = 2.0 if road.highway == 'footway' else 3.5
        amb = '0.55 0.50 0.45 1.0'
        dif = '0.65 0.60 0.55 1.0'
        seg_models = []
        for si in range(n - 1):
            t0 = si / (n - 1)
            t1 = (si + 1) / (n - 1)
            z0 = z_start + (z_end - z_start) * t0
            z1 = z_start + (z_end - z_start) * t1
            e0 = _latlon_to_enu(geom.GetY(si), geom.GetX(si),
                                center_lat, center_lon)
            e1 = _latlon_to_enu(geom.GetY(si + 1), geom.GetX(si + 1),
                                center_lat, center_lon)
            cx = (e0[0] + e1[0]) / 2.0
            cy = (e0[1] + e1[1]) / 2.0
            cz = (z0 + z1) / 2.0
            dx = e1[0] - e0[0]
            dy = e1[1] - e0[1]
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len < 0.1:
                continue
            yaw = math.atan2(dy, dx)
            dz = z1 - z0
            pitch = -math.atan2(dz, seg_len)
            seg_models.append(
                f'    <model name="osm_bridge_{i:04d}_s{si:02d}">\n'
                f'      <static>true</static>\n'
                f'      <pose>{cx:.2f} {cy:.2f} {cz:.2f}'
                f' 0 {pitch:.4f} {yaw:.4f}</pose>\n'
                f'      <link name="link">\n'
                f'        <visual name="visual">\n'
                f'          <geometry>\n'
                f'            <box>\n'
                f'              <size>{seg_len:.2f} {width:.1f}'
                f' {_GANGWAY_THICKNESS}</size>\n'
                f'            </box>\n'
                f'          </geometry>\n'
                f'          <material>\n'
                f'            <ambient>{amb}</ambient>\n'
                f'            <diffuse>{dif}</diffuse>\n'
                f'          </material>\n'
                f'        </visual>\n'
                f'      </link>\n'
                f'    </model>'
            )
        if seg_models:
            osm_bridge_models.extend(seg_models)

    # Build grouped container models
    s57_types = {
        'buildings': s57_buildings,
        'pontoons': pontoons,
        'bridges': bridges,
        'buoys': buoys,
        'beacons': beacons,
        'lights': lights,
        'shore_constructions': shore_constructions,
        'piles': piles,
        'mooring_facilities': mooring_facilities,
        'cranes': cranes,
        'pylons': pylons,
        'coastlines': coastlines,
    }
    # Trees from OSM natural=wood polygons
    tree_models = []
    if osm_natural and 'trees' not in skip and max_trees > 0:
        tree_models = _scatter_trees(
            osm_natural, center_lat, center_lon, terrain, terrain_bounds,
            max_trees=max_trees)
        if tree_models:
            print(f"  Generated {len(tree_models)} trees")

    osm_types = {
        'buildings': osm_buildings,
        'marine': marine_models,
        'bridges': osm_bridge_models,
        'trees': tree_models,
    }

    return {
        's57': _wrap_group('s57_features', s57_types),
        'osm': _wrap_group('osm_features', osm_types),
    }
