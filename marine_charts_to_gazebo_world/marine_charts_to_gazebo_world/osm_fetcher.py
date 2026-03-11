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

"""Fetch OpenStreetMap building and terrain feature data via the Overpass API."""

import hashlib
import json
import logging
import os
import tempfile
from dataclasses import dataclass, field
from typing import List, Optional

import time

import requests
from osgeo import ogr

from .s57_reader import BoundingBox

ogr.UseExceptions()

logger = logging.getLogger(__name__)

# Overpass API endpoints, tried in order.  The primary is the main public
# instance; the others are community mirrors used as fallbacks.
_OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
]

# Retry parameters
_MAX_RETRIES = 3
_RETRY_BACKOFF_BASE = 5  # seconds; actual wait = base * 2^attempt

_DEFAULT_CACHE_DIR = os.path.join(
    os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
    "marine_charts_to_gazebo_world",
)


@dataclass
class OsmBuilding:
    """A building polygon from OpenStreetMap."""

    geometry: ogr.Geometry  # Polygon in WGS84
    height: Optional[float] = None  # meters (from height= tag)
    levels: Optional[int] = None  # from building:levels= tag
    material: str = ''  # from building:material= tag
    colour: str = ''  # from building:colour= tag
    name: str = ''


@dataclass
class OsmLanduse:
    """A landuse polygon from OpenStreetMap."""

    geometry: ogr.Geometry  # Polygon in WGS84
    landuse: str = ''  # e.g. residential, commercial, grass, forest


@dataclass
class OsmRoad:
    """A road linestring from OpenStreetMap."""

    geometry: ogr.Geometry  # LineString in WGS84
    highway: str = ''  # e.g. secondary, residential, service, footway


@dataclass
class OsmParking:
    """A parking area polygon from OpenStreetMap."""

    geometry: ogr.Geometry  # Polygon in WGS84


@dataclass
class OsmNatural:
    """A natural area polygon from OpenStreetMap."""

    geometry: ogr.Geometry  # Polygon in WGS84
    natural: str = ''  # e.g. wood, wetland, beach, scrub


@dataclass
class OsmFeatures:
    """Collection of features fetched from OpenStreetMap."""

    buildings: List[OsmBuilding] = field(default_factory=list)
    landuse: List[OsmLanduse] = field(default_factory=list)
    roads: List[OsmRoad] = field(default_factory=list)
    parking: List[OsmParking] = field(default_factory=list)
    natural: List[OsmNatural] = field(default_factory=list)


def _cache_key(bbox: BoundingBox, terrain: bool = False) -> str:
    """Generate a deterministic cache key from bbox."""
    prefix = "osm_terrain" if terrain else "osm"
    s = f"{prefix}:{bbox.south:.6f},{bbox.west:.6f},{bbox.north:.6f},{bbox.east:.6f}"
    return hashlib.md5(s.encode()).hexdigest()


def _build_overpass_query(bbox: BoundingBox,
                          fetch_terrain_features: bool = False) -> str:
    """Build Overpass QL query for buildings (and terrain features) in bbox."""
    b = f"{bbox.south},{bbox.west},{bbox.north},{bbox.east}"
    if not fetch_terrain_features:
        return (
            "[out:json][timeout:120];\n"
            "(\n"
            f'  way["building"]({b});\n'
            f'  relation["building"]({b});\n'
            ");\n"
            "out geom;\n"
        )
    # Combined query: buildings + landuse + roads + parking + natural
    return (
        "[out:json][timeout:120];\n"
        "(\n"
        f'  way["building"]({b});\n'
        f'  relation["building"]({b});\n'
        f'  way["landuse"]({b});\n'
        f'  way["highway"]({b});\n'
        f'  way["amenity"="parking"]({b});\n'
        f'  way["natural"]({b});\n'
        ");\n"
        "out geom;\n"
    )


def _parse_height(tags: dict) -> Optional[float]:
    """Parse height from OSM tags, returning meters or None."""
    raw = tags.get("height", "")
    if not raw:
        return None
    # Strip trailing 'm' or ' m'
    raw = raw.strip().rstrip("m").strip()
    try:
        return float(raw)
    except ValueError:
        return None


def _parse_levels(tags: dict) -> Optional[int]:
    """Parse building:levels from OSM tags."""
    raw = tags.get("building:levels", "")
    if not raw:
        return None
    try:
        return int(float(raw))
    except ValueError:
        return None


def _coords_to_polygon(coords: list) -> Optional[ogr.Geometry]:
    """Convert a list of {'lat': ..., 'lon': ...} dicts to an OGR Polygon.

    Returns None if fewer than 3 points.
    """
    if len(coords) < 3:
        return None
    ring = ogr.Geometry(ogr.wkbLinearRing)
    for pt in coords:
        ring.AddPoint(pt["lon"], pt["lat"])
    # Close ring if not already closed
    if coords[0]["lat"] != coords[-1]["lat"] or coords[0]["lon"] != coords[-1]["lon"]:
        ring.AddPoint(coords[0]["lon"], coords[0]["lat"])
    poly = ogr.Geometry(ogr.wkbPolygon)
    poly.AddGeometry(ring)
    return poly


def _parse_element(element: dict) -> Optional[OsmBuilding]:
    """Parse a single Overpass JSON element into an OsmBuilding."""
    tags = element.get("tags", {})

    # Only process elements with building tag
    if "building" not in tags:
        return None

    # Get geometry from 'geometry' key (present with 'out geom')
    geom_coords = element.get("geometry")
    if not geom_coords:
        return None

    # For relations, use the first outer member's geometry
    if element.get("type") == "relation":
        members = element.get("members", [])
        outer_coords = None
        for member in members:
            if member.get("role") == "outer" and member.get("geometry"):
                outer_coords = member["geometry"]
                break
        if outer_coords is None:
            return None
        geom_coords = outer_coords

    polygon = _coords_to_polygon(geom_coords)
    if polygon is None or not polygon.IsValid():
        if polygon is not None:
            polygon = polygon.Buffer(0)
            if polygon is None or polygon.IsEmpty():
                return None

    return OsmBuilding(
        geometry=polygon,
        height=_parse_height(tags),
        levels=_parse_levels(tags),
        material=tags.get("building:material", ""),
        colour=tags.get("building:colour", ""),
        name=tags.get("name", ""),
    )


def _coords_to_linestring(coords: list) -> Optional[ogr.Geometry]:
    """Convert a list of {'lat': ..., 'lon': ...} dicts to an OGR LineString.

    Returns None if fewer than 2 points.
    """
    if len(coords) < 2:
        return None
    line = ogr.Geometry(ogr.wkbLineString)
    for pt in coords:
        line.AddPoint(pt["lon"], pt["lat"])
    return line


def _parse_terrain_element(element: dict, features: OsmFeatures):
    """Parse a terrain-related element (landuse, road, parking, natural)."""
    tags = element.get("tags", {})
    geom_coords = element.get("geometry")
    if not geom_coords:
        return

    # Landuse polygons
    if "landuse" in tags and "building" not in tags:
        polygon = _coords_to_polygon(geom_coords)
        if polygon is not None and polygon.IsValid():
            features.landuse.append(OsmLanduse(
                geometry=polygon,
                landuse=tags.get("landuse", ""),
            ))
        return

    # Roads (linestrings)
    if "highway" in tags:
        linestring = _coords_to_linestring(geom_coords)
        if linestring is not None:
            features.roads.append(OsmRoad(
                geometry=linestring,
                highway=tags.get("highway", ""),
            ))
        return

    # Parking areas
    if tags.get("amenity") == "parking":
        polygon = _coords_to_polygon(geom_coords)
        if polygon is not None and polygon.IsValid():
            features.parking.append(OsmParking(geometry=polygon))
        return

    # Natural areas
    if "natural" in tags and "building" not in tags:
        polygon = _coords_to_polygon(geom_coords)
        if polygon is not None and polygon.IsValid():
            features.natural.append(OsmNatural(
                geometry=polygon,
                natural=tags.get("natural", ""),
            ))
        return


def _fetch_with_retries(query: str) -> dict:
    """Fetch Overpass data, retrying across endpoints with backoff.

    Tries each endpoint up to ``_MAX_RETRIES`` times with exponential
    backoff before moving to the next endpoint.  Raises the last
    encountered exception if all endpoints and retries are exhausted.
    """
    last_exc = None
    for endpoint in _OVERPASS_ENDPOINTS:
        for attempt in range(_MAX_RETRIES):
            try:
                logger.info(
                    "Querying %s (attempt %d/%d)...",
                    endpoint, attempt + 1, _MAX_RETRIES,
                )
                response = requests.get(
                    endpoint,
                    params={"data": query},
                    timeout=(30, 180),
                )
                response.raise_for_status()
                return response.json()
            except (requests.RequestException, ValueError) as exc:
                last_exc = exc
                wait = _RETRY_BACKOFF_BASE * (2 ** attempt)
                logger.warning(
                    "%s attempt %d failed: %s  Retrying in %ds...",
                    endpoint, attempt + 1, exc, wait,
                )
                time.sleep(wait)
        logger.warning(
            "All %d retries exhausted for %s, trying next endpoint...",
            _MAX_RETRIES, endpoint,
        )
    raise last_exc  # type: ignore[misc]


def fetch_osm_features(
    bbox: BoundingBox,
    cache_dir: Optional[str] = None,
    fetch_terrain_features: bool = False,
) -> OsmFeatures:
    """Fetch OSM features for a bounding box via Overpass API.

    Args:
        bbox: Geographic bounding box in WGS84 degrees.
        cache_dir: Directory to cache downloaded data. Defaults to
            ~/.cache/marine_charts_to_gazebo_world/.
        fetch_terrain_features: If True, also fetch landuse, roads,
            parking, and natural areas for terrain texturing.

    Returns:
        OsmFeatures with parsed buildings (and terrain features if requested).
    """
    if cache_dir is None:
        cache_dir = _DEFAULT_CACHE_DIR
    os.makedirs(cache_dir, exist_ok=True)

    cache_file = os.path.join(
        cache_dir,
        f"osm_{_cache_key(bbox, terrain=fetch_terrain_features)}.json",
    )

    if os.path.exists(cache_file):
        logger.info("Using cached OSM data: %s", cache_file)
        with open(cache_file) as f:
            data = json.load(f)
    else:
        query = _build_overpass_query(bbox, fetch_terrain_features)
        data = _fetch_with_retries(query)

        # Write cache atomically
        fd, tmp_path = tempfile.mkstemp(
            suffix=".json", dir=cache_dir,
        )
        try:
            with os.fdopen(fd, "w") as tmp_file:
                json.dump(data, tmp_file)
            os.rename(tmp_path, cache_file)
        except Exception:
            try:
                os.unlink(tmp_path)
            except FileNotFoundError:
                pass
            raise
        logger.info("Cached OSM data to %s", cache_file)

    # Parse elements
    features = OsmFeatures()
    for element in data.get("elements", []):
        building = _parse_element(element)
        if building is not None:
            features.buildings.append(building)
        if fetch_terrain_features:
            _parse_terrain_element(element, features)

    return features
