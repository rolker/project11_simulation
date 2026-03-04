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

"""Read S57 ENC chart features relevant to world generation."""

import logging
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from osgeo import gdal, ogr

ogr.UseExceptions()

logger = logging.getLogger(__name__)


def _gdal_error_handler(err_class, err_num, err_msg):
    """Route GDAL messages through Python logging, suppressing known noise."""
    if 'Illegal feature attribute id' in err_msg:
        return
    if err_class == gdal.CE_Warning:
        logger.warning('GDAL: %s', err_msg.rstrip())
    elif err_class == gdal.CE_Failure:
        logger.error('GDAL: %s', err_msg.rstrip())


@dataclass
class BoundingBox:
    """Geographic bounding box in WGS84 degrees."""

    south: float
    west: float
    north: float
    east: float

    @property
    def center_lat(self):
        return (self.south + self.north) / 2.0

    @property
    def center_lon(self):
        return (self.west + self.east) / 2.0


@dataclass
class DepthArea:
    """DEPARE polygon with min/max depth values."""

    min_depth: float  # DRVAL1 (meters, positive down)
    max_depth: float  # DRVAL2 (meters, positive down)
    geometry: ogr.Geometry  # Polygon in WGS84
    compilation_scale: float = 0.0  # DSPM_CSCL (lower = more detailed)


@dataclass
class Sounding:
    """Individual depth sounding point."""

    lat: float
    lon: float
    depth: float  # meters, positive down


@dataclass
class Building:
    """Building or structure polygon footprint (BUISGL, LNDMRK, SILTNK)."""

    geometry: ogr.Geometry  # Polygon in WGS84
    objl: int = 12  # S57 OBJL code (for height defaults in feature_models)
    lnam: str = ''  # S57 Long Name (unique within a chart)
    scamin: float = float('inf')  # Scale minimum (lower = more detailed)
    objnam: str = ''  # Feature name (for debugging)
    osm_height: Optional[float] = None  # from OSM height or levels*3
    osm_material: str = ''  # from OSM building:material
    osm_colour: str = ''  # from OSM building:colour


@dataclass
class Pontoon:
    """Pontoon polygon footprint (PONTON)."""

    geometry: ogr.Geometry  # Polygon in WGS84


@dataclass
class Bridge:
    """Bridge polygon footprint (BRIDGE) with optional vertical clearance."""

    geometry: ogr.Geometry  # Polygon in WGS84
    clearance: float = 0.0  # VERCLR vertical clearance (meters)


@dataclass
class Buoy:
    """Lateral buoy point feature (BOYLAT)."""

    lat: float
    lon: float
    colour: int = 0  # S57 COLOUR attribute (1=white, 3=red, 4=green, 6=yellow)


@dataclass
class Beacon:
    """Special purpose beacon point feature (BCNSPP)."""

    lat: float
    lon: float


@dataclass
class Light:
    """Light point feature (LIGHTS)."""

    lat: float
    lon: float


@dataclass
class S57Features:
    """Collection of features extracted from S57 charts."""

    depth_areas: List[DepthArea] = field(default_factory=list)
    soundings: List[Sounding] = field(default_factory=list)
    land_areas: List[ogr.Geometry] = field(default_factory=list)
    coastlines: List[ogr.Geometry] = field(default_factory=list)
    buildings: List[Building] = field(default_factory=list)
    pontoons: List[Pontoon] = field(default_factory=list)
    bridges: List[Bridge] = field(default_factory=list)
    buoys: List[Buoy] = field(default_factory=list)
    beacons: List[Beacon] = field(default_factory=list)
    lights: List[Light] = field(default_factory=list)


def _polygon_area_m2(geom: ogr.Geometry, center_lat_rad: float) -> float:
    """Approximate polygon area in m² using WGS84 radii of curvature.

    Uses the geometry's GetArea() in deg² and scales by local metric factors.
    """
    from marine_autonomy.wgs84 import M, N
    area_deg2 = abs(geom.GetArea())
    m_lat = M(center_lat_rad)  # meters per radian in lat
    n_lon = N(center_lat_rad) * math.cos(center_lat_rad)  # meters per radian in lon
    # deg² → rad² → m²
    rad_per_deg = math.pi / 180.0
    return area_deg2 * (rad_per_deg ** 2) * m_lat * n_lon


# Maximum area (m²) for LNDMRK/SILTNK polygons to be treated as buildings
_MAX_LANDMARK_AREA_M2 = 5_000.0


@dataclass
class _ChartInfo:
    """Metadata for a single S57 chart used during coverage filtering."""

    filepath: str
    compilation_scale: float  # DSPM_CSCL (lower = more detailed)
    coverage: ogr.Geometry  # Polygon/MultiPolygon of chart coverage in WGS84


def _bbox_to_polygon(bbox: BoundingBox) -> ogr.Geometry:
    """Convert a BoundingBox to an OGR Polygon geometry."""
    ring = ogr.Geometry(ogr.wkbLinearRing)
    ring.AddPoint(bbox.west, bbox.south)
    ring.AddPoint(bbox.east, bbox.south)
    ring.AddPoint(bbox.east, bbox.north)
    ring.AddPoint(bbox.west, bbox.north)
    ring.AddPoint(bbox.west, bbox.south)
    poly = ogr.Geometry(ogr.wkbPolygon)
    poly.AddGeometry(ring)
    return poly


def _repair_geometry(geom: ogr.Geometry) -> ogr.Geometry:
    """Repair an invalid geometry using Buffer(0)."""
    if geom is not None and not geom.IsValid():
        repaired = geom.Buffer(0)
        if repaired is not None:
            return repaired
    return geom


def _scan_chart_coverage(
    filepath: str, bbox_poly: ogr.Geometry,
) -> Optional[_ChartInfo]:
    """Scan a single .000 file for its compilation scale and coverage area.

    Reads M_COVR features: CATCOV=1 polygons form coverage, CATCOV=2 are
    subtracted as holes. Falls back to dataset spatial extent if M_COVR
    is absent. Returns None if the chart doesn't intersect the bbox.
    """
    gdal.PushErrorHandler(_gdal_error_handler)
    try:
        ds = ogr.Open(filepath, 0)
        if ds is None:
            return None

        try:
            # Read compilation scale
            compilation_scale = 0.0
            dsid_layer = ds.GetLayerByName('DSID')
            if dsid_layer is not None:
                dsid_layer.ResetReading()
                dsid_feat = dsid_layer.GetNextFeature()
                if dsid_feat is not None:
                    cscl_idx = dsid_feat.GetFieldIndex('DSPM_CSCL')
                    if cscl_idx >= 0:
                        compilation_scale = dsid_feat.GetFieldAsDouble(
                            cscl_idx
                        )

            # Build coverage from M_COVR layer
            coverage_add = ogr.Geometry(ogr.wkbMultiPolygon)
            coverage_sub = ogr.Geometry(ogr.wkbMultiPolygon)
            mcovr_layer = ds.GetLayerByName('M_COVR')
            if mcovr_layer is not None:
                mcovr_layer.ResetReading()
                for feat in mcovr_layer:
                    geom = feat.GetGeometryRef()
                    if geom is None:
                        continue
                    catcov_idx = feat.GetFieldIndex('CATCOV')
                    catcov = (
                        feat.GetFieldAsInteger(catcov_idx)
                        if catcov_idx >= 0 else 1
                    )
                    if catcov == 2:
                        coverage_sub.AddGeometry(geom.Clone())
                    else:
                        coverage_add.AddGeometry(geom.Clone())

            if coverage_add.GetGeometryCount() > 0:
                coverage = _repair_geometry(coverage_add.UnionCascaded())
                if coverage_sub.GetGeometryCount() > 0:
                    sub_union = _repair_geometry(
                        coverage_sub.UnionCascaded()
                    )
                    coverage = _repair_geometry(
                        coverage.Difference(sub_union)
                    )
            else:
                # Fallback: use dataset spatial extent
                # Try the first feature layer's extent
                extent = None
                for li in range(ds.GetLayerCount()):
                    lyr = ds.GetLayerByIndex(li)
                    try:
                        ext = lyr.GetExtent()
                        if ext is not None:
                            if extent is None:
                                extent = list(ext)
                            else:
                                extent[0] = min(extent[0], ext[0])
                                extent[1] = max(extent[1], ext[1])
                                extent[2] = min(extent[2], ext[2])
                                extent[3] = max(extent[3], ext[3])
                    except Exception:
                        continue
                if extent is None:
                    return None
                ring = ogr.Geometry(ogr.wkbLinearRing)
                ring.AddPoint(extent[0], extent[2])
                ring.AddPoint(extent[1], extent[2])
                ring.AddPoint(extent[1], extent[3])
                ring.AddPoint(extent[0], extent[3])
                ring.AddPoint(extent[0], extent[2])
                coverage = ogr.Geometry(ogr.wkbPolygon)
                coverage.AddGeometry(ring)

            # Intersect with bbox
            if coverage is None or coverage.IsEmpty():
                return None
            clipped = coverage.Intersection(bbox_poly)
            if clipped is None or clipped.IsEmpty():
                return None

            return _ChartInfo(
                filepath=filepath,
                compilation_scale=compilation_scale,
                coverage=_repair_geometry(clipped),
            )
        finally:
            ds = None
    finally:
        gdal.PopErrorHandler()


def _build_effective_areas(
    charts: List[_ChartInfo],
) -> List[Tuple[str, ogr.Geometry, float]]:
    """Compute non-overlapping effective areas, most detailed charts first.

    Returns list of (filepath, effective_area, compilation_scale) tuples.
    The most detailed chart (lowest scale) gets its full coverage; coarser
    charts only get the area not already covered by more detailed charts.
    """
    # Sort by compilation_scale ascending (most detailed first)
    sorted_charts = sorted(charts, key=lambda c: c.compilation_scale)

    result = []
    already_covered = ogr.Geometry(ogr.wkbMultiPolygon)  # empty

    for chart in sorted_charts:
        if already_covered.IsEmpty():
            effective = chart.coverage.Clone()
        else:
            effective = chart.coverage.Difference(already_covered)
            effective = _repair_geometry(effective)

        if effective is None or effective.IsEmpty():
            logger.debug(
                "Chart %s (scale %.0f) fully covered by more detailed charts",
                os.path.basename(chart.filepath),
                chart.compilation_scale,
            )
            continue

        result.append((chart.filepath, effective, chart.compilation_scale))

        # Union full coverage (not just effective) to block coarser charts
        already_covered = _repair_geometry(
            already_covered.Union(chart.coverage)
        )

    return result


def _clip_features_to_area(
    features: S57Features, effective_area: ogr.Geometry,
) -> S57Features:
    """Clip all features to an effective coverage area.

    Polygons/lines are geometrically intersected; points are tested for
    containment.
    """
    clipped = S57Features()

    # Depth areas (polygons)
    for da in features.depth_areas:
        g = da.geometry.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                clipped.depth_areas.append(DepthArea(
                    min_depth=da.min_depth,
                    max_depth=da.max_depth,
                    geometry=g.Clone(),
                    compilation_scale=da.compilation_scale,
                ))

    # Land areas (polygons)
    for la in features.land_areas:
        g = la.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                clipped.land_areas.append(g.Clone())

    # Coastlines (linestrings)
    for cl in features.coastlines:
        g = cl.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbLineString, ogr.wkbMultiLineString):
                clipped.coastlines.append(g.Clone())
            elif gtype == ogr.wkbGeometryCollection:
                # Extract only line geometries from collection
                for i in range(g.GetGeometryCount()):
                    sub = g.GetGeometryRef(i)
                    stype = sub.GetGeometryType() & 0xFF
                    if stype in (ogr.wkbLineString, ogr.wkbMultiLineString):
                        clipped.coastlines.append(sub.Clone())

    # Soundings (points)
    for s in features.soundings:
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(s.lon, s.lat)
        if effective_area.Contains(pt):
            clipped.soundings.append(s)

    # Buildings (polygons)
    for b in features.buildings:
        g = b.geometry.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                clipped.buildings.append(Building(
                    geometry=g.Clone(),
                    objl=b.objl,
                    lnam=b.lnam,
                    scamin=b.scamin,
                    objnam=b.objnam,
                    osm_height=b.osm_height,
                    osm_material=b.osm_material,
                    osm_colour=b.osm_colour,
                ))

    # Pontoons (polygons)
    for p in features.pontoons:
        g = p.geometry.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                clipped.pontoons.append(Pontoon(geometry=g.Clone()))

    # Bridges (polygons)
    for br in features.bridges:
        g = br.geometry.Intersection(effective_area)
        g = _repair_geometry(g)
        if g is not None and not g.IsEmpty():
            gtype = g.GetGeometryType() & 0xFF
            if gtype in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                clipped.bridges.append(Bridge(
                    geometry=g.Clone(), clearance=br.clearance,
                ))

    # Point features: buoys, beacons, lights
    for buoy in features.buoys:
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(buoy.lon, buoy.lat)
        if effective_area.Contains(pt):
            clipped.buoys.append(buoy)

    for beacon in features.beacons:
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(beacon.lon, beacon.lat)
        if effective_area.Contains(pt):
            clipped.beacons.append(beacon)

    for light in features.lights:
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(light.lon, light.lat)
        if effective_area.Contains(pt):
            clipped.lights.append(light)

    return clipped


def _clip_geometry(geom: ogr.Geometry, bbox: BoundingBox) -> Optional[ogr.Geometry]:
    """Clip a geometry to a bounding box, returning None if no intersection."""
    ring = ogr.Geometry(ogr.wkbLinearRing)
    ring.AddPoint(bbox.west, bbox.south)
    ring.AddPoint(bbox.east, bbox.south)
    ring.AddPoint(bbox.east, bbox.north)
    ring.AddPoint(bbox.west, bbox.north)
    ring.AddPoint(bbox.west, bbox.south)
    clip_poly = ogr.Geometry(ogr.wkbPolygon)
    clip_poly.AddGeometry(ring)

    intersection = geom.Intersection(clip_poly)
    if intersection is None or intersection.IsEmpty():
        return None
    return intersection


def _extract_soundings_from_multipoint(
    geom: ogr.Geometry, bbox: BoundingBox
) -> List[Sounding]:
    """Extract sounding points from a SOUNDG multipoint geometry.

    S57 SOUNDG features store depth as the Z coordinate of 3D points.
    Each feature may contain a single point or a multipoint geometry.
    """
    soundings = []
    geom_type = geom.GetGeometryType()

    if geom_type in (ogr.wkbPoint, ogr.wkbPoint25D):
        lon, lat = geom.GetX(), geom.GetY()
        if (bbox.south <= lat <= bbox.north and bbox.west <= lon <= bbox.east):
            depth = geom.GetZ() if geom.GetCoordinateDimension() >= 3 else 0.0
            soundings.append(Sounding(lat=lat, lon=lon, depth=depth))
    elif geom_type in (ogr.wkbMultiPoint, ogr.wkbMultiPoint25D):
        for i in range(geom.GetGeometryCount()):
            pt = geom.GetGeometryRef(i)
            lon, lat = pt.GetX(), pt.GetY()
            if (bbox.south <= lat <= bbox.north and bbox.west <= lon <= bbox.east):
                depth = pt.GetZ() if pt.GetCoordinateDimension() >= 3 else 0.0
                soundings.append(Sounding(lat=lat, lon=lon, depth=depth))

    return soundings


def _extract_point(geom: ogr.Geometry) -> Optional[tuple]:
    """Extract (lat, lon) from a point geometry, or None."""
    geom_type = geom.GetGeometryType() & 0xFF  # strip 25D flag
    if geom_type == ogr.wkbPoint:
        return (geom.GetY(), geom.GetX())
    return None


def read_s57_file(filepath: str, bbox: BoundingBox) -> S57Features:
    """Read S57 features from a single .000 file within a bounding box.

    Args:
        filepath: Path to S57 .000 file.
        bbox: Geographic bounding box to clip features to.

    Returns:
        S57Features with extracted depth areas, soundings, land, coastlines.
    """
    features = S57Features()

    gdal.PushErrorHandler(_gdal_error_handler)
    try:
        ds = ogr.Open(filepath, 0)
        if ds is None:
            raise FileNotFoundError(f"Cannot open S57 file: {filepath}")

        try:
            # Read compilation scale from DSID layer (once per chart)
            compilation_scale = 0.0
            dsid_layer = ds.GetLayerByName('DSID')
            if dsid_layer is not None:
                dsid_layer.ResetReading()
                dsid_feat = dsid_layer.GetNextFeature()
                if dsid_feat is not None:
                    cscl_idx = dsid_feat.GetFieldIndex('DSPM_CSCL')
                    if cscl_idx >= 0:
                        compilation_scale = dsid_feat.GetFieldAsDouble(
                            cscl_idx
                        )

            for layer_idx in range(ds.GetLayerCount()):
                layer = ds.GetLayerByIndex(layer_idx)
                layer.SetSpatialFilterRect(
                    bbox.west, bbox.south, bbox.east, bbox.north
                )
                layer.ResetReading()

                for feature in layer:
                    objl_idx = feature.GetFieldIndex("OBJL")
                    if objl_idx == -1:
                        continue
                    objl = feature.GetFieldAsInteger(objl_idx)
                    geom = feature.GetGeometryRef()
                    if geom is None:
                        continue

                    if objl == 42:  # DEPARE
                        drval1_idx = feature.GetFieldIndex("DRVAL1")
                        drval2_idx = feature.GetFieldIndex("DRVAL2")
                        if drval1_idx >= 0 and drval2_idx >= 0:
                            min_depth = feature.GetFieldAsDouble(drval1_idx)
                            max_depth = feature.GetFieldAsDouble(drval2_idx)
                            clipped = _clip_geometry(geom, bbox)
                            if clipped is not None:
                                features.depth_areas.append(
                                    DepthArea(
                                        min_depth=min_depth,
                                        max_depth=max_depth,
                                        geometry=clipped.Clone(),
                                        compilation_scale=compilation_scale,
                                    )
                                )

                    elif objl == 129:  # SOUNDG
                        features.soundings.extend(
                            _extract_soundings_from_multipoint(geom, bbox)
                        )

                    elif objl == 71:  # LNDARE
                        clipped = _clip_geometry(geom, bbox)
                        if clipped is not None:
                            features.land_areas.append(clipped.Clone())

                    elif objl == 30:  # COALNE
                        clipped = _clip_geometry(geom, bbox)
                        if clipped is not None:
                            features.coastlines.append(clipped.Clone())

                    elif objl in (12, 73, 119):  # BUISGL, LNDMRK, SILTNK
                        clipped = _clip_geometry(geom, bbox)
                        if clipped is not None:
                            # Filter oversized LNDMRK/SILTNK polygons
                            if objl in (73, 119):
                                center_lat_rad = math.radians(
                                    bbox.center_lat
                                )
                                area = _polygon_area_m2(
                                    clipped, center_lat_rad
                                )
                                if area > _MAX_LANDMARK_AREA_M2:
                                    objnam = ''
                                    idx = feature.GetFieldIndex("OBJNAM")
                                    if idx >= 0:
                                        objnam = (
                                            feature.GetFieldAsString(idx)
                                            or ''
                                        )
                                    logger.debug(
                                        "Skipping large OBJL %d '%s' "
                                        "(%.0f m²)",
                                        objl, objnam, area,
                                    )
                                    continue
                            # Extract metadata
                            lnam = ''
                            lnam_idx = feature.GetFieldIndex("LNAM")
                            if lnam_idx >= 0:
                                lnam = (
                                    feature.GetFieldAsString(lnam_idx)
                                    or ''
                                )
                            scamin = float('inf')
                            scamin_idx = feature.GetFieldIndex("SCAMIN")
                            if scamin_idx >= 0:
                                val = feature.GetFieldAsDouble(scamin_idx)
                                if val > 0:
                                    scamin = val
                            objnam = ''
                            objnam_idx = feature.GetFieldIndex("OBJNAM")
                            if objnam_idx >= 0:
                                objnam = (
                                    feature.GetFieldAsString(objnam_idx)
                                    or ''
                                )
                            features.buildings.append(
                                Building(
                                    geometry=clipped.Clone(),
                                    objl=objl,
                                    lnam=lnam,
                                    scamin=scamin,
                                    objnam=objnam,
                                )
                            )

                    elif objl == 95:  # PONTON
                        clipped = _clip_geometry(geom, bbox)
                        if clipped is not None:
                            features.pontoons.append(
                                Pontoon(geometry=clipped.Clone())
                            )

                    elif objl == 11:  # BRIDGE
                        clipped = _clip_geometry(geom, bbox)
                        if clipped is not None:
                            verclr = 0.0
                            verclr_idx = feature.GetFieldIndex("VERCLR")
                            if verclr_idx >= 0:
                                verclr = feature.GetFieldAsDouble(
                                    verclr_idx
                                )
                            features.bridges.append(
                                Bridge(
                                    geometry=clipped.Clone(),
                                    clearance=verclr,
                                )
                            )

                    elif objl == 17:  # BOYLAT
                        pt = _extract_point(geom)
                        if pt is not None:
                            colour = 0
                            colour_idx = feature.GetFieldIndex("COLOUR")
                            if colour_idx >= 0:
                                colour = feature.GetFieldAsInteger(
                                    colour_idx
                                )
                            features.buoys.append(
                                Buoy(lat=pt[0], lon=pt[1], colour=colour)
                            )

                    elif objl == 9:  # BCNSPP
                        pt = _extract_point(geom)
                        if pt is not None:
                            features.beacons.append(
                                Beacon(lat=pt[0], lon=pt[1])
                            )

                    elif objl == 75:  # LIGHTS
                        pt = _extract_point(geom)
                        if pt is not None:
                            features.lights.append(
                                Light(lat=pt[0], lon=pt[1])
                            )
        finally:
            ds = None
    finally:
        gdal.PopErrorHandler()

    return features


def read_enc_directory(enc_root: str, bbox: BoundingBox) -> S57Features:
    """Read all S57 .000 files under an ENC directory within a bounding box.

    Uses coverage-based filtering: when multiple charts overlap, the most
    detailed chart (lowest compilation scale) wins. Coarser charts only
    contribute features in areas not covered by more detailed charts.

    Args:
        enc_root: Root directory containing S57 ENC files.
        bbox: Geographic bounding box to clip features to.

    Returns:
        Merged S57Features from all charts intersecting the bounding box.
    """
    bbox_poly = _bbox_to_polygon(bbox)

    # Pass 1: Scan all charts for coverage and scale
    chart_files = []
    for dirpath, _dirnames, filenames in os.walk(enc_root):
        for filename in filenames:
            if filename.upper().endswith(".000"):
                chart_files.append(os.path.join(dirpath, filename))

    charts = []
    for filepath in chart_files:
        try:
            info = _scan_chart_coverage(filepath, bbox_poly)
            if info is not None:
                charts.append(info)
                logger.debug(
                    "Chart %s: scale=%.0f",
                    os.path.basename(filepath),
                    info.compilation_scale,
                )
        except Exception as e:
            logger.warning("Skipping coverage scan of %s: %s", filepath, e)

    if not charts:
        return S57Features()

    # Compute non-overlapping effective areas
    effective_areas = _build_effective_areas(charts)
    logger.info(
        "Coverage filtering: %d charts scanned, %d contribute features",
        len(charts), len(effective_areas),
    )

    # Pass 2: Read features and clip to effective areas
    combined = S57Features()
    for i, (filepath, effective_area, scale) in enumerate(effective_areas):
        try:
            features = read_s57_file(filepath, bbox)
            # Skip clipping for the most detailed chart (first in list) —
            # its effective area equals its full coverage, and read_s57_file
            # already clips to bbox
            if i == 0:
                clipped = features
            else:
                clipped = _clip_features_to_area(features, effective_area)
            combined.depth_areas.extend(clipped.depth_areas)
            combined.soundings.extend(clipped.soundings)
            combined.land_areas.extend(clipped.land_areas)
            combined.coastlines.extend(clipped.coastlines)
            combined.buildings.extend(clipped.buildings)
            combined.pontoons.extend(clipped.pontoons)
            combined.bridges.extend(clipped.bridges)
            combined.buoys.extend(clipped.buoys)
            combined.beacons.extend(clipped.beacons)
            combined.lights.extend(clipped.lights)
        except Exception as e:
            logger.warning("Skipping %s: %s", filepath, e)

    # Deduplicate BUISGL buildings by LNAM (keep lowest SCAMIN)
    combined.buildings = _dedup_buildings(combined.buildings)

    return combined


def _dedup_buildings(buildings: List[Building]) -> List[Building]:
    """Remove duplicate buildings that share the same LNAM.

    For BUISGL features appearing in multiple chart scales, keep the
    record with the lowest SCAMIN (most detailed chart). Buildings
    without LNAM are always kept.
    """
    best_by_lnam: dict[str, Building] = {}
    no_lnam: list[Building] = []

    for b in buildings:
        if not b.lnam:
            no_lnam.append(b)
            continue
        existing = best_by_lnam.get(b.lnam)
        if existing is None or b.scamin < existing.scamin:
            best_by_lnam[b.lnam] = b

    n_dupes = len(buildings) - len(best_by_lnam) - len(no_lnam)
    if n_dupes > 0:
        logger.info("Removed %d duplicate buildings by LNAM", n_dupes)

    return no_lnam + list(best_by_lnam.values())
