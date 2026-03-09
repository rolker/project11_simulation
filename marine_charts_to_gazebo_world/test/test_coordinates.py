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

"""Tests for coordinates module."""

import numpy as np
import pytest
from osgeo import ogr

from marine_charts_to_gazebo_world.coordinates import (
    enu_to_latlon,
    enu_to_latlon_array,
    latlon_to_enu,
    latlon_to_enu_array,
    transform_geometries_to_enu,
)

REF_LAT = 43.075
REF_LON = -70.71


class TestLatLonToEnu:

    def test_center_is_origin(self):
        """Reference point should map to (0, 0)."""
        x, y = latlon_to_enu(REF_LAT, REF_LON, REF_LAT, REF_LON)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)

    def test_north_offset(self):
        """Point north of reference should have positive northing."""
        x, y = latlon_to_enu(REF_LAT + 0.001, REF_LON, REF_LAT, REF_LON)
        assert x == pytest.approx(0.0, abs=0.01)
        assert y > 0
        assert y == pytest.approx(111.32, abs=1.0)

    def test_east_offset(self):
        """Point east of reference should have positive easting."""
        x, y = latlon_to_enu(REF_LAT, REF_LON + 0.001, REF_LAT, REF_LON)
        assert y == pytest.approx(0.0, abs=0.01)
        assert x > 0


class TestEnuToLatLon:

    def test_origin_roundtrip(self):
        """ENU origin should map back to reference point."""
        lat, lon = enu_to_latlon(0.0, 0.0, REF_LAT, REF_LON)
        assert lat == pytest.approx(REF_LAT, abs=1e-8)
        assert lon == pytest.approx(REF_LON, abs=1e-8)

    def test_roundtrip_north(self):
        """Forward then inverse should recover original lat/lon."""
        orig_lat = REF_LAT + 0.01
        orig_lon = REF_LON
        e, n = latlon_to_enu(orig_lat, orig_lon, REF_LAT, REF_LON)
        lat, lon = enu_to_latlon(e, n, REF_LAT, REF_LON)
        assert lat == pytest.approx(orig_lat, abs=1e-6)
        assert lon == pytest.approx(orig_lon, abs=1e-6)

    def test_roundtrip_diagonal(self):
        """Roundtrip for a point offset in both E and N."""
        orig_lat = REF_LAT + 0.005
        orig_lon = REF_LON + 0.005
        e, n = latlon_to_enu(orig_lat, orig_lon, REF_LAT, REF_LON)
        lat, lon = enu_to_latlon(e, n, REF_LAT, REF_LON)
        assert lat == pytest.approx(orig_lat, abs=1e-6)
        assert lon == pytest.approx(orig_lon, abs=1e-6)


class TestArrayVersions:

    def test_array_matches_scalar(self):
        """Array version should match scalar for multiple points."""
        lats = np.array([REF_LAT, REF_LAT + 0.01, REF_LAT - 0.005])
        lons = np.array([REF_LON, REF_LON + 0.01, REF_LON - 0.005])

        for i in range(len(lats)):
            e_scalar, n_scalar = latlon_to_enu(
                lats[i], lons[i], REF_LAT, REF_LON,
            )
            e_array, n_array = latlon_to_enu_array(
                lats, lons, REF_LAT, REF_LON,
            )
            assert e_array[i] == pytest.approx(e_scalar, abs=0.01)
            assert n_array[i] == pytest.approx(n_scalar, abs=0.01)

    def test_inverse_array_roundtrip(self):
        """enu_to_latlon_array should roundtrip with latlon_to_enu_array."""
        lats = np.array([REF_LAT + 0.01, REF_LAT - 0.01])
        lons = np.array([REF_LON + 0.01, REF_LON - 0.01])
        east, north = latlon_to_enu_array(lats, lons, REF_LAT, REF_LON)
        lats_out, lons_out = enu_to_latlon_array(
            east, north, REF_LAT, REF_LON,
        )
        np.testing.assert_allclose(lats_out, lats, atol=1e-6)
        np.testing.assert_allclose(lons_out, lons, atol=1e-6)


class TestTransformGeometries:

    def test_polygon_transform(self):
        """Polygon vertices should be in ENU meters after transform."""
        ring = ogr.Geometry(ogr.wkbLinearRing)
        ring.AddPoint(REF_LON, REF_LAT)
        ring.AddPoint(REF_LON + 0.001, REF_LAT)
        ring.AddPoint(REF_LON + 0.001, REF_LAT + 0.001)
        ring.AddPoint(REF_LON, REF_LAT + 0.001)
        ring.AddPoint(REF_LON, REF_LAT)
        poly = ogr.Geometry(ogr.wkbPolygon)
        poly.AddGeometry(ring)

        result = transform_geometries_to_enu([poly], REF_LAT, REF_LON)
        assert len(result) == 1
        out_ring = result[0].GetGeometryRef(0)
        # First vertex should be near origin
        assert out_ring.GetX(0) == pytest.approx(0.0, abs=0.1)
        assert out_ring.GetY(0) == pytest.approx(0.0, abs=0.1)
        # Other vertices should be in meters (order of 80-111m for 0.001 deg)
        assert 50 < out_ring.GetX(1) < 150
        assert 50 < out_ring.GetY(2) < 150

    def test_linestring_transform(self):
        """LineString should transform correctly."""
        line = ogr.Geometry(ogr.wkbLineString)
        line.AddPoint(REF_LON, REF_LAT)
        line.AddPoint(REF_LON + 0.001, REF_LAT)

        result = transform_geometries_to_enu([line], REF_LAT, REF_LON)
        assert len(result) == 1
        assert result[0].GetPointCount() == 2
        assert result[0].GetX(0) == pytest.approx(0.0, abs=0.1)

    def test_point_transform(self):
        """Point should transform correctly."""
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint(REF_LON, REF_LAT)

        result = transform_geometries_to_enu([pt], REF_LAT, REF_LON)
        assert len(result) == 1
        assert result[0].GetX() == pytest.approx(0.0, abs=0.1)
        assert result[0].GetY() == pytest.approx(0.0, abs=0.1)
