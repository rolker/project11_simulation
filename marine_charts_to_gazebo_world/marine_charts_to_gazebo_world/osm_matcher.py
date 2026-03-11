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

"""Match S57 buildings to OSM buildings and enrich with OSM metadata."""

import logging
import math

from osgeo import ogr

from .osm_fetcher import OsmFeatures
from .s57_reader import Building, S57Features

ogr.UseExceptions()

logger = logging.getLogger(__name__)

# Maximum distance (degrees) between centroids to consider a match.
# ~50m at mid-latitudes ≈ 0.00045 degrees.
_MAX_CENTROID_DISTANCE_DEG = 0.00045

# Minimum IoU to accept a match.
_MIN_IOU = 0.3


def _centroid_distance_deg(geom_a: ogr.Geometry, geom_b: ogr.Geometry) -> float:
    """Euclidean distance between centroids in degrees."""
    ca = geom_a.Centroid()
    cb = geom_b.Centroid()
    dx = ca.GetX() - cb.GetX()
    dy = ca.GetY() - cb.GetY()
    return math.sqrt(dx * dx + dy * dy)


def _iou(geom_a: ogr.Geometry, geom_b: ogr.Geometry) -> float:
    """Compute intersection-over-union of two geometries."""
    try:
        intersection = geom_a.Intersection(geom_b)
        if intersection is None or intersection.IsEmpty():
            return 0.0
        inter_area = intersection.GetArea()
        union = geom_a.Union(geom_b)
        if union is None or union.IsEmpty():
            return 0.0
        union_area = union.GetArea()
        if union_area == 0.0:
            return 0.0
        return inter_area / union_area
    except Exception:
        return 0.0


def match_and_enrich(
    s57_features: S57Features,
    osm_features: OsmFeatures,
    add_unmatched: bool = True,
) -> tuple:
    """Match S57 buildings to OSM buildings and enrich with OSM data.

    For each S57 building, finds the best OSM match by centroid proximity
    and IoU. When matched, copies OSM height, material, and colour onto
    the S57 Building dataclass, and replaces the footprint polygon.

    When *add_unmatched* is True (the default), unmatched OSM buildings
    are converted to S57 Building objects and appended to the buildings
    list.

    Args:
        s57_features: S57Features with buildings to enrich.
        osm_features: OsmFeatures with OSM buildings to match against.
        add_unmatched: If True, add unmatched OSM buildings as new
            S57 Building objects. Set False to only enrich existing
            S57 buildings without adding new ones.

    Returns:
        Tuple of (enriched S57Features, number of matched buildings,
        number of added OSM-only buildings).
    """
    if not osm_features.buildings:
        return s57_features, 0, 0

    # Track which OSM buildings get matched
    matched_osm_indices = set()
    n_matched = 0

    for building in s57_features.buildings:
        best_iou = 0.0
        best_osm = None
        best_idx = -1

        for idx, osm_building in enumerate(osm_features.buildings):
            # Quick centroid distance filter
            dist = _centroid_distance_deg(
                building.geometry, osm_building.geometry,
            )
            if dist > _MAX_CENTROID_DISTANCE_DEG:
                continue

            score = _iou(building.geometry, osm_building.geometry)
            if score > best_iou:
                best_iou = score
                best_osm = osm_building
                best_idx = idx

        if best_osm is not None and best_iou >= _MIN_IOU:
            matched_osm_indices.add(best_idx)

            # Enrich with OSM data
            if best_osm.height is not None:
                building.osm_height = best_osm.height
            elif best_osm.levels is not None:
                building.osm_height = best_osm.levels * 3.0

            building.osm_material = best_osm.material
            building.osm_colour = best_osm.colour

            # Replace footprint with OSM polygon (typically more accurate)
            building.geometry = best_osm.geometry.Clone()

            n_matched += 1
            logger.debug(
                "Matched S57 building '%s' to OSM '%s' (IoU=%.2f)",
                building.objnam or building.lnam,
                best_osm.name,
                best_iou,
            )

    # Add unmatched OSM buildings as new Building objects
    n_added = 0
    if not add_unmatched:
        return s57_features, n_matched, n_added
    for idx, osm_building in enumerate(osm_features.buildings):
        if idx in matched_osm_indices:
            continue

        osm_height = None
        if osm_building.height is not None:
            osm_height = osm_building.height
        elif osm_building.levels is not None:
            osm_height = osm_building.levels * 3.0

        s57_features.buildings.append(Building(
            geometry=osm_building.geometry.Clone(),
            objl=12,  # generic building
            objnam=osm_building.name,
            osm_height=osm_height,
            osm_material=osm_building.material,
            osm_colour=osm_building.colour,
            osm_only=True,
        ))
        n_added += 1

    return s57_features, n_matched, n_added
