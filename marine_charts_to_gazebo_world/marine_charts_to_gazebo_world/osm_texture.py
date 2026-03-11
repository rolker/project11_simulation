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

"""Rasterize OSM terrain features onto a PIL Image for heightmap texturing."""

import logging
from typing import Optional

import numpy as np
from PIL import Image, ImageDraw

from .coordinates import latlon_to_enu_array
from .osm_fetcher import OsmFeatures

logger = logging.getLogger(__name__)

# Base land color (muted green-brown, matching the original placeholder)
LAND_BASE = (120, 140, 95)

# Landuse colors by type
_LANDUSE_COLORS = {
    "grass": (100, 160, 80),
    "meadow": (110, 165, 85),
    "recreation_ground": (100, 160, 80),
    "residential": (175, 170, 160),
    "commercial": (200, 190, 170),
    "retail": (200, 185, 165),
    "industrial": (185, 180, 175),
    "farmland": (180, 195, 140),
    "farmyard": (185, 180, 150),
    "forest": (70, 120, 55),
    "cemetery": (130, 155, 110),
    "military": (160, 155, 140),
}

# Natural area colors by type
_NATURAL_COLORS = {
    "wood": (60, 110, 50),
    "scrub": (85, 130, 65),
    "heath": (130, 150, 100),
    "grassland": (110, 165, 85),
    "wetland": (80, 140, 130),
    "marsh": (80, 140, 130),
    "beach": (210, 200, 170),
    "sand": (210, 200, 170),
    "bare_rock": (170, 165, 155),
    "water": (100, 140, 180),
}

# Parking color
_PARKING_COLOR = (130, 130, 130)

# Road colors and widths (meters) by highway type
_ROAD_STYLES = {
    "motorway": {"color": (140, 140, 140), "width": 12.0},
    "trunk": {"color": (140, 140, 140), "width": 10.0},
    "primary": {"color": (150, 150, 145), "width": 9.0},
    "secondary": {"color": (155, 155, 150), "width": 8.0},
    "tertiary": {"color": (160, 160, 155), "width": 7.0},
    "residential": {"color": (165, 165, 160), "width": 6.0},
    "unclassified": {"color": (165, 165, 160), "width": 5.0},
    "service": {"color": (170, 170, 165), "width": 4.0},
    "track": {"color": (175, 170, 155), "width": 3.0},
    "footway": {"color": (180, 175, 165), "width": 1.5},
    "path": {"color": (180, 175, 165), "width": 1.5},
    "cycleway": {"color": (180, 175, 165), "width": 2.0},
    "steps": {"color": (175, 170, 160), "width": 1.5},
}
_DEFAULT_ROAD_STYLE = {"color": (170, 170, 165), "width": 4.0}

# Marine infrastructure colors and widths by man_made type
_MAN_MADE_STYLES = {
    "pier": {"color": (180, 175, 165), "width": 3.0},       # light gray
    "quay": {"color": (160, 155, 150), "width": 4.0},       # medium gray
    "breakwater": {"color": (140, 140, 135), "width": 5.0},  # dark gray
    "groyne": {"color": (140, 140, 135), "width": 3.0},      # dark gray
}
_DEFAULT_MAN_MADE_STYLE = {"color": (160, 155, 150), "width": 3.0}


def _latlon_to_pixel(lats, lons, ref_lat, ref_lon, tex_size, size_x, size_y,
                     grid_size):
    """Convert lat/lon arrays to pixel coordinates.

    The texture is a square image (grid_size x grid_size) that covers
    tex_size x tex_size meters (where tex_size = max(size_x, size_y)).
    The heightmap extent (size_x x size_y) is centered within this
    square so that Gazebo's UV mapping aligns features correctly.

    Args:
        lats: numpy array of latitudes.
        lons: numpy array of longitudes.
        ref_lat: Reference latitude (terrain center).
        ref_lon: Reference longitude (terrain center).
        tex_size: Texture tiling size in meters (square).
        size_x: Terrain width in meters.
        size_y: Terrain height in meters.
        grid_size: Image size in pixels.

    Returns:
        List of (px, py) tuples suitable for PIL drawing.
    """
    east, north = latlon_to_enu_array(
        np.asarray(lats, dtype=np.float64),
        np.asarray(lons, dtype=np.float64),
        ref_lat, ref_lon,
    )
    # Map ENU to UV coordinates, then to pixels.
    # Gazebo heightmap UV: U goes [0, size_x/tex_size] west-to-east,
    # V goes [0, size_y/tex_size] south-to-north.
    # Image row 0 = top (V=1), row grid_size-1 = bottom (V=0 = south).
    px = (east + size_x / 2) / tex_size * (grid_size - 1)
    py = (grid_size - 1) - (north + size_y / 2) / tex_size * (grid_size - 1)
    return list(zip(px.tolist(), py.tolist()))


def _geometry_to_pixel_coords(geom, ref_lat, ref_lon,
                              tex_size, size_x, size_y, grid_size):
    """Extract coordinates from an OGR geometry and convert to pixels.

    For polygons, returns the exterior ring coordinates.
    For linestrings, returns the point coordinates.
    """
    geom_type = geom.GetGeometryType() & 0xFF

    if geom_type == 2:  # LineString
        lats = [geom.GetY(i) for i in range(geom.GetPointCount())]
        lons = [geom.GetX(i) for i in range(geom.GetPointCount())]
        return _latlon_to_pixel(lats, lons, ref_lat, ref_lon,
                                tex_size, size_x, size_y, grid_size)

    if geom_type == 3:  # Polygon
        ring = geom.GetGeometryRef(0)  # exterior ring
        if ring is None:
            return []
        lats = [ring.GetY(i) for i in range(ring.GetPointCount())]
        lons = [ring.GetX(i) for i in range(ring.GetPointCount())]
        return _latlon_to_pixel(lats, lons, ref_lat, ref_lon,
                                tex_size, size_x, size_y, grid_size)

    return []


def rasterize_osm_texture(
    osm_features: OsmFeatures,
    terrain_info: dict,
    grid_size: int,
    ref_lat: float,
    ref_lon: float,
    terrain: Optional[np.ndarray] = None,
) -> Image.Image:
    """Rasterize OSM features onto a texture image for heightmap land areas.

    Renders features back-to-front:
    1. Base land color
    2. Landuse polygons
    3. Natural area polygons
    4. Parking polygons
    5. Roads with appropriate widths

    Only pixels above sea level (elevation > 0) are rendered when terrain
    data is provided.

    Args:
        osm_features: Parsed OSM features with landuse, roads, parking, natural.
        terrain_info: dict from build_terrain() with size_x, size_y.
        grid_size: Image dimensions (grid_size x grid_size pixels).
        ref_lat: Reference latitude for coordinate conversion.
        ref_lon: Reference longitude for coordinate conversion.
        terrain: Optional terrain elevation array for water masking.

    Returns:
        PIL Image (RGB) of the rasterized texture.
    """
    size_x = terrain_info["size_x"]
    size_y = terrain_info["size_y"]

    # Gazebo heightmap texture <size> uses a single value for both U and V
    # tiling.  For non-square heightmaps we use the larger dimension so the
    # texture covers the full extent without clipping.  Features are
    # rasterized into the correct region of this square texture.
    tex_size = max(size_x, size_y)

    # Start with base land color
    img = Image.new("RGB", (grid_size, grid_size), LAND_BASE)
    draw = ImageDraw.Draw(img)

    # Layer 1: Landuse polygons
    for lu in osm_features.landuse:
        color = _LANDUSE_COLORS.get(lu.landuse)
        if color is None:
            continue
        coords = _geometry_to_pixel_coords(
            lu.geometry, ref_lat, ref_lon,
            tex_size, size_x, size_y, grid_size)
        if len(coords) >= 3:
            draw.polygon(coords, fill=color)

    # Layer 2: Natural polygons
    for nat in osm_features.natural:
        color = _NATURAL_COLORS.get(nat.natural)
        if color is None:
            continue
        coords = _geometry_to_pixel_coords(
            nat.geometry, ref_lat, ref_lon,
            tex_size, size_x, size_y, grid_size)
        if len(coords) >= 3:
            draw.polygon(coords, fill=color)

    # Layer 3: Parking polygons
    for pkg in osm_features.parking:
        coords = _geometry_to_pixel_coords(
            pkg.geometry, ref_lat, ref_lon,
            tex_size, size_x, size_y, grid_size)
        if len(coords) >= 3:
            draw.polygon(coords, fill=_PARKING_COLOR)

    # Layer 4: Marine infrastructure (man_made polygons and linestrings)
    meters_per_pixel = tex_size / (grid_size - 1)
    for mm in osm_features.man_made:
        style = _MAN_MADE_STYLES.get(mm.man_made, _DEFAULT_MAN_MADE_STYLE)
        geom_type = mm.geometry.GetGeometryType() & 0xFF
        coords = _geometry_to_pixel_coords(
            mm.geometry, ref_lat, ref_lon,
            tex_size, size_x, size_y, grid_size)
        if geom_type == 3 and len(coords) >= 3:  # Polygon
            draw.polygon(coords, fill=style["color"])
        elif len(coords) >= 2:  # LineString
            pixel_width = max(1, round(style["width"] / meters_per_pixel))
            draw.line(coords, fill=style["color"], width=pixel_width)

    # Layer 5: Roads with width
    for road in osm_features.roads:
        style = _ROAD_STYLES.get(road.highway, _DEFAULT_ROAD_STYLE)
        pixel_width = max(1, round(style["width"] / meters_per_pixel))
        coords = _geometry_to_pixel_coords(
            road.geometry, ref_lat, ref_lon,
            tex_size, size_x, size_y, grid_size)
        if len(coords) >= 2:
            draw.line(coords, fill=style["color"], width=pixel_width)

    # Apply water mask: reset underwater pixels to base color
    if terrain is not None:
        arr = np.array(img)
        water_mask = terrain <= 0.0
        arr[water_mask] = LAND_BASE
        img = Image.fromarray(arr)

    n_features = (len(osm_features.landuse) + len(osm_features.natural)
                  + len(osm_features.parking) + len(osm_features.roads)
                  + len(osm_features.man_made))
    logger.info("Rasterized %d terrain features onto %dx%d texture",
                n_features, grid_size, grid_size)

    return img, tex_size
