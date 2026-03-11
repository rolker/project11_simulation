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

"""Match S57 features to OSM features and enrich with OSM metadata."""

import logging
import math

from osgeo import ogr

from .osm_fetcher import OsmFeatures
from .s57_reader import Building, S57Features, ShoreCon

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


# --- Pier matching ---

# Maximum centroid distance for pier matching (~100m at mid-latitudes).
_MAX_PIER_CENTROID_DISTANCE_DEG = 0.0009

# Buffer around S57 linestring (~2m in degrees) to create an area for overlap.
_PIER_LINE_BUFFER_DEG = 0.00002

# Minimum fraction of the buffered S57 line that must fall inside the OSM
# polygon to accept a match.
_MIN_PIER_OVERLAP = 0.3


def _line_polygon_overlap(line: ogr.Geometry, polygon: ogr.Geometry) -> float:
    """Fraction of a buffered linestring that overlaps with a polygon."""
    try:
        buffered = line.Buffer(_PIER_LINE_BUFFER_DEG)
        if buffered is None or buffered.IsEmpty():
            return 0.0
        buffered_area = buffered.GetArea()
        if buffered_area == 0.0:
            return 0.0
        intersection = buffered.Intersection(polygon)
        if intersection is None or intersection.IsEmpty():
            return 0.0
        return intersection.GetArea() / buffered_area
    except Exception:
        return 0.0


def match_piers(
    s57_features: S57Features,
    osm_features: OsmFeatures,
    add_unmatched: bool = True,
) -> tuple:
    """Match S57 SLCONS piers to OSM man_made=pier polygons.

    For each S57 SLCONS with catslc=4 (pier/jetty), finds the best OSM
    pier polygon match using centroid proximity and buffered-line overlap.
    When matched, stores the OSM polygon as ``osm_geometry`` on the
    ShoreCon for polygon rendering.

    When *add_unmatched* is True, unmatched OSM pier polygons are added
    as new ShoreCon objects with ``osm_only=True``.

    Returns:
        Tuple of (enriched S57Features, n_matched, n_added, set of
        matched OSM man_made indices).
    """
    # Collect OSM pier polygons (with their index into osm_features.man_made)
    osm_piers = []
    for idx, mm in enumerate(osm_features.man_made):
        if mm.man_made != 'pier':
            continue
        geom_type = mm.geometry.GetGeometryType() & 0xFF
        if geom_type not in (3, 6):  # only Polygon / MultiPolygon
            continue
        osm_piers.append((idx, mm))

    if not osm_piers:
        return s57_features, 0, 0, set()

    matched_osm_indices = set()
    n_matched = 0

    for sc in s57_features.shore_constructions:
        if sc.catslc != 4:
            continue
        # Only match linestring S57 piers (polygons are already good)
        geom_type = sc.geometry.GetGeometryType() & 0xFF
        if geom_type not in (2, 5, 7):  # LineString, MultiLineString, Collection
            continue

        best_overlap = 0.0
        best_osm_idx = -1
        best_osm_geom = None

        for osm_idx, osm_mm in osm_piers:
            dist = _centroid_distance_deg(sc.geometry, osm_mm.geometry)
            if dist > _MAX_PIER_CENTROID_DISTANCE_DEG:
                continue

            overlap = _line_polygon_overlap(sc.geometry, osm_mm.geometry)
            if overlap > best_overlap:
                best_overlap = overlap
                best_osm_idx = osm_idx
                best_osm_geom = osm_mm.geometry

        if best_osm_geom is not None and best_overlap >= _MIN_PIER_OVERLAP:
            matched_osm_indices.add(best_osm_idx)
            sc.osm_geometry = best_osm_geom.Clone()
            n_matched += 1
            logger.debug(
                "Matched S57 pier to OSM pier (overlap=%.2f)", best_overlap,
            )

    # Add unmatched OSM piers as new ShoreCon objects
    n_added = 0
    if add_unmatched:
        for osm_idx, osm_mm in osm_piers:
            if osm_idx in matched_osm_indices:
                continue
            s57_features.shore_constructions.append(ShoreCon(
                geometry=osm_mm.geometry.Clone(),
                catslc=4,
                osm_only=True,
            ))
            n_added += 1

    return s57_features, n_matched, n_added, matched_osm_indices
