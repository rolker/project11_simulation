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

"""Fetch OpenStreetMap building data via the Overpass API."""

import hashlib
import json
import logging
import os
import tempfile
from dataclasses import dataclass, field
from typing import List, Optional

import requests
from osgeo import ogr

from .s57_reader import BoundingBox

ogr.UseExceptions()

logger = logging.getLogger(__name__)

_OVERPASS_URL = "https://overpass-api.de/api/interpreter"

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
class OsmFeatures:
    """Collection of features fetched from OpenStreetMap."""

    buildings: List[OsmBuilding] = field(default_factory=list)


def _cache_key(bbox: BoundingBox) -> str:
    """Generate a deterministic cache key from bbox."""
    s = f"osm:{bbox.south:.6f},{bbox.west:.6f},{bbox.north:.6f},{bbox.east:.6f}"
    return hashlib.md5(s.encode()).hexdigest()


def _build_overpass_query(bbox: BoundingBox) -> str:
    """Build Overpass QL query for buildings, piers, and bridges."""
    b = f"{bbox.south},{bbox.west},{bbox.north},{bbox.east}"
    return (
        "[out:json][timeout:120];\n"
        "(\n"
        f'  way["building"]({b});\n'
        f'  relation["building"]({b});\n'
        f'  way["man_made"="pier"]({b});\n'
        f'  way["man_made"="bridge"]({b});\n'
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


def fetch_osm_features(
    bbox: BoundingBox,
    cache_dir: Optional[str] = None,
) -> OsmFeatures:
    """Fetch OSM building features for a bounding box via Overpass API.

    Args:
        bbox: Geographic bounding box in WGS84 degrees.
        cache_dir: Directory to cache downloaded data. Defaults to
            ~/.cache/marine_charts_to_gazebo_world/.

    Returns:
        OsmFeatures with parsed buildings.
    """
    if cache_dir is None:
        cache_dir = _DEFAULT_CACHE_DIR
    os.makedirs(cache_dir, exist_ok=True)

    cache_file = os.path.join(cache_dir, f"osm_{_cache_key(bbox)}.json")

    if os.path.exists(cache_file):
        logger.info("Using cached OSM data: %s", cache_file)
        with open(cache_file) as f:
            data = json.load(f)
    else:
        query = _build_overpass_query(bbox)
        logger.info("Querying Overpass API...")
        response = requests.get(
            _OVERPASS_URL,
            params={"data": query},
            timeout=(30, 180),
        )
        response.raise_for_status()
        data = response.json()

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

    return features
