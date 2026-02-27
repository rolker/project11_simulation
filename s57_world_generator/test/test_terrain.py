"""Tests for terrain module."""

import numpy as np
import pytest

from s57_world_generator.s57_reader import BoundingBox, Sounding, S57Features
from s57_world_generator.terrain import build_terrain, _sample_raster


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


class TestBuildTerrain:
    def test_output_shape(self):
        """Output grid should have the requested grid_size."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        # Simple 3x3 base elevation
        base = np.array([[-5.0, -10.0, -5.0],
                         [-10.0, -15.0, -10.0],
                         [-5.0, -10.0, -5.0]])
        # GeoTransform covering the bbox
        gt = (-71.0, 0.005, 0.0, 43.01, 0.0, -0.005)

        terrain, info = build_terrain(
            bbox=bbox,
            base_elevation=base,
            base_geotransform=gt,
            grid_size=17,  # 2^4 + 1
        )
        assert terrain.shape == (17, 17)

    def test_output_range(self):
        """Terrain info should report correct min/max elevation."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        base = np.array([[-5.0, -10.0], [-15.0, -20.0]])
        gt = (-71.0, 0.01, 0.0, 43.01, 0.0, -0.01)

        terrain, info = build_terrain(
            bbox=bbox,
            base_elevation=base,
            base_geotransform=gt,
            grid_size=5,  # 2^2 + 1
        )
        assert info["min_elevation"] <= info["max_elevation"]
        assert info["size_x"] > 0
        assert info["size_y"] > 0

    def test_with_soundings(self):
        """Soundings should refine the terrain."""
        bbox = BoundingBox(south=43.0, west=-71.0, north=43.01, east=-70.99)
        # Flat seafloor at -10m
        base = np.full((3, 3), -10.0)
        gt = (-71.0, 0.005, 0.0, 43.01, 0.0, -0.005)

        # Add a sounding showing -20m at center
        features = S57Features(
            soundings=[
                Sounding(lat=43.005, lon=-70.995, depth=20.0),
            ]
        )

        terrain, info = build_terrain(
            bbox=bbox,
            base_elevation=base,
            base_geotransform=gt,
            s57_features=features,
            grid_size=5,
        )
        # Terrain should still be valid
        assert terrain.shape == (5, 5)
        assert not np.any(np.isnan(terrain))
