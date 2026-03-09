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

"""Tests for terrain module."""

import numpy as np
import pytest
from osgeo import ogr

from marine_charts_to_gazebo_world.s57_reader import (
    BoundingBox, DepthArea, Sounding, S57Features,
)
from marine_charts_to_gazebo_world.terrain import build_terrain, _sample_raster


class TestSampleRaster:
    def test_identity_sampling(self):
        """Sampling at grid points should return exact values."""
        data = np.array([[10.0, 20.0], [30.0, 40.0]])
        # GeoTransform: (x_origin, x_pixel_size, 0, y_origin, 0, y_pixel_size)
        gt = (0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

        lons = np.array([[0.0, 1.0]])
        lats = np.array([[0.0, 0.0]])

        result = _sample_raster(data, gt, lons, lats)
        assert result[0, 0] == pytest.approx(10.0)
        assert result[0, 1] == pytest.approx(20.0)

    def test_interpolation(self):
        """Sampling between grid points should interpolate."""
        data = np.array([[0.0, 100.0], [0.0, 100.0]])
        gt = (0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

        lons = np.array([[0.5]])
        lats = np.array([[0.0]])

        result = _sample_raster(data, gt, lons, lats)
        assert result[0, 0] == pytest.approx(50.0)


class TestBuildTerrainWithBase:
    """Tests for ETOPO-based terrain (base_elevation provided)."""

    def test_output_shape(self):
        """Output grid should have the requested grid_size."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        base = np.array([[-5.0, -10.0, -5.0],
                         [-10.0, -15.0, -10.0],
                         [-5.0, -10.0, -5.0]])
        gt = (-71.0, 0.005, 0.0, 43.01, 0.0, -0.005)

        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            base_elevation=base,
            base_geotransform=gt,
            grid_size=17,
        )
        assert terrain.shape == (17, 17)

    def test_output_range(self):
        """Terrain info should report correct min/max elevation."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        base = np.array([[-5.0, -10.0], [-15.0, -20.0]])
        gt = (-71.0, 0.01, 0.0, 43.01, 0.0, -0.01)

        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            base_elevation=base,
            base_geotransform=gt,
            grid_size=5,
        )
        assert info["min_elevation"] <= info["max_elevation"]
        assert info["size_x"] > 0
        assert info["size_y"] > 0

    def test_with_soundings(self):
        """Soundings should refine the terrain."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        base = np.full((3, 3), -10.0)
        gt = (-71.0, 0.005, 0.0, 43.01, 0.0, -0.005)

        features = S57Features(
            soundings=[
                Sounding(lat=43.005, lon=-70.995, depth=20.0),
            ]
        )

        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            base_elevation=base,
            base_geotransform=gt,
            s57_features=features,
            grid_size=5,
        )
        assert terrain.shape == (5, 5)
        assert not np.any(np.isnan(terrain))


def _make_polygon(coords):
    """Create an OGR polygon from a list of (lon, lat) tuples."""
    ring = ogr.Geometry(ogr.wkbLinearRing)
    for lon, lat in coords:
        ring.AddPoint(lon, lat)
    ring.AddPoint(coords[0][0], coords[0][1])
    poly = ogr.Geometry(ogr.wkbPolygon)
    poly.AddGeometry(ring)
    return poly


def _make_linestring(coords):
    """Create an OGR linestring from a list of (lon, lat) tuples."""
    line = ogr.Geometry(ogr.wkbLineString)
    for lon, lat in coords:
        line.AddPoint(lon, lat)
    return line


class TestBuildS57Terrain:
    """Tests for S57-only terrain (no base elevation)."""

    def test_no_features_flat(self):
        """No S57 features should produce a flat surface at 0."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            grid_size=9,
        )
        assert terrain.shape == (9, 9)
        assert np.all(terrain == 0.0)

    def test_land_area_positive_elevation(self):
        """Land area should have positive elevation."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        # Land polygon covering the center of the bbox
        land_poly = _make_polygon([
            (-70.995, 43.008),
            (-70.985, 43.008),
            (-70.985, 43.012),
            (-70.995, 43.012),
        ])
        features = S57Features(land_areas=[land_poly])
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=33,
        )
        assert terrain.shape == (33, 33)
        # Land cells should have positive elevation
        assert info["max_elevation"] > 0.0
        # Water cells (no soundings) should be at 0
        assert info["min_elevation"] == pytest.approx(0.0)

    def test_land_ramp_caps_at_max(self):
        """Land elevation should cap at the maximum ramp height."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        # Large land polygon covering most of the bbox
        land_poly = _make_polygon([
            (-70.999, 43.001),
            (-70.981, 43.001),
            (-70.981, 43.019),
            (-70.999, 43.019),
        ])
        features = S57Features(land_areas=[land_poly])
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=65,
        )
        # Interior land cells far from shore should reach 5.0m cap
        assert info["max_elevation"] == pytest.approx(5.0)

    def test_soundings_provide_water_depth(self):
        """Water cells with soundings should have negative elevation."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        # Land in the north half
        land_poly = _make_polygon([
            (-70.999, 43.011),
            (-70.981, 43.011),
            (-70.981, 43.019),
            (-70.999, 43.019),
        ])
        # Soundings in the south half (water)
        soundings = [
            Sounding(lat=43.003, lon=-70.995, depth=5.0),
            Sounding(lat=43.003, lon=-70.985, depth=10.0),
            Sounding(lat=43.007, lon=-70.990, depth=7.0),
        ]
        features = S57Features(
            land_areas=[land_poly], soundings=soundings,
        )
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=33,
        )
        # Should have both positive (land) and negative (water) elevation
        assert info["max_elevation"] > 0.0
        assert info["min_elevation"] < 0.0

    def test_coastline_marks_land(self):
        """Coastline pixels should be treated as land."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        coastline = _make_linestring([
            (-70.999, 43.010),
            (-70.981, 43.010),
        ])
        features = S57Features(coastlines=[coastline])
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=33,
        )
        # Coastline pixels should have non-negative elevation
        assert info["max_elevation"] >= 0.0


class TestDepthAreaClamping:
    """Tests for depth area clamping of interpolated water depths."""

    def test_depth_clamped_to_depare_range(self):
        """Interpolated depth outside DRVAL range gets clamped."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        # Depth area covering the entire bbox: 3-8m deep
        da_poly = _make_polygon([
            (-71.0, 43.0),
            (-70.98, 43.0),
            (-70.98, 43.02),
            (-71.0, 43.02),
        ])
        depth_area = DepthArea(
            min_depth=3.0, max_depth=8.0,
            geometry=da_poly, compilation_scale=50000.0,
        )
        # Soundings spread across the area — one very deep (15m),
        # one very shallow (1m), both outside the DEPARE range
        soundings = [
            Sounding(lat=43.005, lon=-70.995, depth=15.0),
            Sounding(lat=43.005, lon=-70.985, depth=1.0),
            Sounding(lat=43.015, lon=-70.995, depth=15.0),
            Sounding(lat=43.015, lon=-70.985, depth=1.0),
        ]
        features = S57Features(
            depth_areas=[depth_area], soundings=soundings,
        )
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=33,
        )
        # All cells should be clamped to DEPARE range:
        # depths 3-8m -> elevation -8 to -3
        water_cells = terrain[terrain < 0]
        assert len(water_cells) > 0, "Should have water cells"
        assert np.min(water_cells) >= -8.0 - 0.01
        assert np.max(water_cells) <= -3.0 + 0.01

    def test_scale_ordering_detailed_wins(self):
        """Overlapping depth areas: more detailed chart wins."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.02, east=-70.98)
        # Coarse chart: whole bbox, 5-20m
        coarse_poly = _make_polygon([
            (-70.999, 43.001),
            (-70.981, 43.001),
            (-70.981, 43.019),
            (-70.999, 43.019),
        ])
        coarse_da = DepthArea(
            min_depth=5.0, max_depth=20.0,
            geometry=coarse_poly, compilation_scale=100000.0,
        )
        # Detailed chart: center area, 8-12m (tighter range)
        detail_poly = _make_polygon([
            (-70.995, 43.006),
            (-70.985, 43.006),
            (-70.985, 43.014),
            (-70.995, 43.014),
        ])
        detail_da = DepthArea(
            min_depth=8.0, max_depth=12.0,
            geometry=detail_poly, compilation_scale=20000.0,
        )
        # Soundings at 15m everywhere — in the detailed area they should
        # be clamped to 12m, in the coarse-only area to 20m
        soundings = [
            Sounding(lat=43.005, lon=-70.995, depth=15.0),
            Sounding(lat=43.005, lon=-70.985, depth=15.0),
            Sounding(lat=43.015, lon=-70.995, depth=15.0),
            Sounding(lat=43.015, lon=-70.985, depth=15.0),
        ]
        features = S57Features(
            depth_areas=[coarse_da, detail_da], soundings=soundings,
        )
        terrain, info = build_terrain(
            bbox=bbox,
            ref_lat=bbox.center_lat,
            ref_lon=bbox.center_lon,
            s57_features=features,
            grid_size=33,
        )
        # Center of grid (within detailed chart area) should use 8-12m range
        center = terrain[16, 16]
        # The 15m sounding should be clamped to -12m elevation (12m depth)
        assert center >= -12.0 - 0.01
        assert center <= -8.0 + 0.01
