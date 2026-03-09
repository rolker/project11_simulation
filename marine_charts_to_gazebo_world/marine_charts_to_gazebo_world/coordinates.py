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

"""Shared coordinate transformations between WGS84 and local ENU.

All conversions use ECEF as the intermediate frame, via
``marine_autonomy.wgs84``, to avoid the cosine-approximation errors
that accumulate over large bounding boxes.
"""

import math

import numpy as np
from marine_autonomy.wgs84 import (
    toECEFfromDegrees,
    fromECEFtoLatLongDegrees,
)
from osgeo import ogr

# WGS84 ellipsoid constants (same as marine_autonomy.wgs84)
_A = 6378137.0
_E2 = 0.006694380004260814


def latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """Convert WGS84 lat/lon to local ENU meters via ECEF.

    Args:
        lat: Latitude in degrees.
        lon: Longitude in degrees.
        ref_lat: Reference (origin) latitude in degrees.
        ref_lon: Reference (origin) longitude in degrees.

    Returns:
        (east, north) in meters relative to the reference point.
    """
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


def latlon_to_enu_array(lats, lons, ref_lat, ref_lon):
    """Vectorized conversion of WGS84 lat/lon arrays to local ENU meters.

    Args:
        lats: numpy array of latitudes in degrees.
        lons: numpy array of longitudes in degrees.
        ref_lat: Reference latitude in degrees.
        ref_lon: Reference longitude in degrees.

    Returns:
        (east, north) numpy arrays in meters.
    """
    lats_r = np.radians(lats)
    lons_r = np.radians(lons)
    ref_lat_r = math.radians(ref_lat)
    ref_lon_r = math.radians(ref_lon)

    # Transverse radius of curvature
    sin_lat = np.sin(lats_r)
    n = _A / np.sqrt(1 - _E2 * sin_lat ** 2)

    # ECEF for query points
    cos_lat = np.cos(lats_r)
    cos_lon = np.cos(lons_r)
    sin_lon = np.sin(lons_r)
    px = n * cos_lat * cos_lon
    py = n * cos_lat * sin_lon
    pz = n * (1 - _E2) * sin_lat

    # ECEF for reference point
    rx, ry, rz = toECEFfromDegrees(ref_lat, ref_lon)

    dx = px - rx
    dy = py - ry
    dz = pz - rz

    # Rotation to ENU
    sin_ref_lat = math.sin(ref_lat_r)
    cos_ref_lat = math.cos(ref_lat_r)
    sin_ref_lon = math.sin(ref_lon_r)
    cos_ref_lon = math.cos(ref_lon_r)

    east = -sin_ref_lon * dx + cos_ref_lon * dy
    north = (-sin_ref_lat * cos_ref_lon * dx
             - sin_ref_lat * sin_ref_lon * dy
             + cos_ref_lat * dz)
    return east, north


def enu_to_latlon(east, north, ref_lat, ref_lon):
    """Convert local ENU meters back to WGS84 lat/lon via ECEF.

    Args:
        east: Easting in meters.
        north: Northing in meters.
        ref_lat: Reference latitude in degrees.
        ref_lon: Reference longitude in degrees.

    Returns:
        (lat, lon) in degrees.
    """
    rx, ry, rz = toECEFfromDegrees(ref_lat, ref_lon)
    lat_r = math.radians(ref_lat)
    lon_r = math.radians(ref_lon)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lon = math.sin(lon_r)
    cos_lon = math.cos(lon_r)

    # ENU to ECEF delta (inverse of the rotation matrix)
    # up = 0 (stay on surface)
    dx = (-sin_lon * east
          - sin_lat * cos_lon * north)
    dy = (cos_lon * east
          - sin_lat * sin_lon * north)
    dz = cos_lat * north

    lat, lon, _ = fromECEFtoLatLongDegrees(rx + dx, ry + dy, rz + dz)
    return lat, lon


def enu_to_latlon_array(east, north, ref_lat, ref_lon):
    """Vectorized conversion of ENU meters to WGS84 lat/lon.

    Args:
        east: numpy array of easting values in meters.
        north: numpy array of northing values in meters.
        ref_lat: Reference latitude in degrees.
        ref_lon: Reference longitude in degrees.

    Returns:
        (lats, lons) numpy arrays in degrees.
    """
    rx, ry, rz = toECEFfromDegrees(ref_lat, ref_lon)
    lat_r = math.radians(ref_lat)
    lon_r = math.radians(ref_lon)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lon = math.sin(lon_r)
    cos_lon = math.cos(lon_r)

    # ENU to ECEF delta
    dx = -sin_lon * east - sin_lat * cos_lon * north
    dy = cos_lon * east - sin_lat * sin_lon * north
    dz = cos_lat * north

    # ECEF coordinates
    ex = rx + dx
    ey = ry + dy
    ez = rz + dz

    # Vectorized ECEF to lat/lon (Bowring's iterative method simplified)
    r = np.sqrt(ex ** 2 + ey ** 2)
    lon_out = np.degrees(np.arctan2(ey, ex))

    # Use the same closed-form as wgs84.fromECEFtoLatLong but vectorized
    b = 6356752.3142
    ep2 = (_A * _A) / (b * b) - 1.0
    E2_val = _A * _A - b * b
    F = 54 * b * b * ez * ez
    G = r * r + (1 - _E2) * ez * ez - _E2 * E2_val
    C = (_E2 * _E2 * F * r * r) / (G ** 3)
    s = np.cbrt(1 + C + np.sqrt(C * C + 2 * C))
    P = F / (3 * (s + 1.0 / s + 1) ** 2 * G * G)
    Q = np.sqrt(1 + 2 * _E2 * _E2 * P)
    r0 = (-(P * _E2 * r) / (1 + Q) +
          np.sqrt(0.5 * _A * _A * (1 + 1 / Q)
                  - (P * (1 - _E2) * ez * ez) / (Q * (1 + Q))
                  - 0.5 * P * r * r))
    U = np.sqrt((r - _E2 * r0) ** 2 + ez * ez)
    V = np.sqrt((r - _E2 * r0) ** 2 + (1 - _E2) * ez * ez)
    Z0 = b * b * ez / (_A * V)
    lat_out = np.degrees(np.arctan((ez + ep2 * Z0) / r))

    return lat_out, lon_out


def transform_geometries_to_enu(geometries, ref_lat, ref_lon):
    """Convert a list of OGR geometries from WGS84 to ENU meters.

    Each geometry is cloned and its vertices are transformed in-place
    from (lon, lat) degrees to (east, north) meters.

    Args:
        geometries: List of OGR Geometry objects in WGS84.
        ref_lat: Reference latitude in degrees.
        ref_lon: Reference longitude in degrees.

    Returns:
        List of new OGR Geometry objects with coordinates in ENU meters.
    """
    result = []
    for geom in geometries:
        result.append(_transform_geometry(geom, ref_lat, ref_lon))
    return result


def _transform_geometry(geom, ref_lat, ref_lon):
    """Recursively transform an OGR geometry to ENU meters."""
    geom_type = geom.GetGeometryType() & 0xFF

    if geom_type in (1,):  # Point
        lon, lat = geom.GetX(), geom.GetY()
        east, north = latlon_to_enu(lat, lon, ref_lat, ref_lon)
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(east, north)
        return pt

    if geom_type in (2,):  # LineString
        line = ogr.Geometry(ogr.wkbLineString)
        for i in range(geom.GetPointCount()):
            lon, lat = geom.GetX(i), geom.GetY(i)
            east, north = latlon_to_enu(lat, lon, ref_lat, ref_lon)
            line.AddPoint(east, north)
        return line

    if geom_type in (3,):  # Polygon
        poly = ogr.Geometry(ogr.wkbPolygon)
        for ring_idx in range(geom.GetGeometryCount()):
            ring_in = geom.GetGeometryRef(ring_idx)
            ring_out = ogr.Geometry(ogr.wkbLinearRing)
            for i in range(ring_in.GetPointCount()):
                lon, lat = ring_in.GetX(i), ring_in.GetY(i)
                east, north = latlon_to_enu(lat, lon, ref_lat, ref_lon)
                ring_out.AddPoint(east, north)
            poly.AddGeometry(ring_out)
        return poly

    # Multi/collection types: recurse
    if geom_type in (4, 5, 6, 7):
        multi = ogr.Geometry(geom.GetGeometryType())
        for i in range(geom.GetGeometryCount()):
            multi.AddGeometry(
                _transform_geometry(geom.GetGeometryRef(i), ref_lat, ref_lon)
            )
        return multi

    # Fallback: clone as-is
    return geom.Clone()
