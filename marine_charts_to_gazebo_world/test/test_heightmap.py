"""Tests for heightmap module."""

import os
import tempfile

import numpy as np
import pytest
from PIL import Image

from marine_charts_to_gazebo_world.heightmap import terrain_to_heightmap


class TestTerrainToHeightmap:
    def test_output_files_created(self):
        """Should create heightmap.png, model.sdf, model.config."""
        terrain = np.random.uniform(-20, 5, (17, 17))  # 2^4 + 1
        info = {"size_x": 1000.0, "size_y": 1000.0, "grid_size": 17}

        with tempfile.TemporaryDirectory() as tmpdir:
            result = terrain_to_heightmap(terrain, info, tmpdir)

            model_dir = os.path.join(tmpdir, "terrain")
            assert os.path.exists(os.path.join(model_dir, "heightmap.png"))
            assert os.path.exists(os.path.join(model_dir, "model.sdf"))
            assert os.path.exists(os.path.join(model_dir, "model.config"))

    def test_heightmap_dimensions(self):
        """Output PNG should match input grid dimensions."""
        grid_size = 33  # 2^5 + 1
        terrain = np.random.uniform(-20, 5, (grid_size, grid_size))
        info = {"size_x": 1000.0, "size_y": 1000.0, "grid_size": grid_size}

        with tempfile.TemporaryDirectory() as tmpdir:
            result = terrain_to_heightmap(terrain, info, tmpdir)

            img = Image.open(result["heightmap_path"])
            assert img.size == (grid_size, grid_size)

    def test_heightmap_16bit(self):
        """Output PNG should be 16-bit."""
        terrain = np.random.uniform(-20, 5, (17, 17))
        info = {"size_x": 500.0, "size_y": 500.0, "grid_size": 17}

        with tempfile.TemporaryDirectory() as tmpdir:
            result = terrain_to_heightmap(terrain, info, tmpdir)

            img = Image.open(result["heightmap_path"])
            assert img.mode in ("I;16", "I")

    def test_elevation_range_in_result(self):
        """Result dict should contain correct elevation range."""
        terrain = np.array([[0.0, 10.0], [5.0, -5.0]])
        terrain = np.array([
            [0.0, 5.0, 10.0],
            [2.0, -5.0, 8.0],
            [1.0, 3.0, 6.0],
        ])
        info = {"size_x": 100.0, "size_y": 100.0, "grid_size": 3}

        with tempfile.TemporaryDirectory() as tmpdir:
            result = terrain_to_heightmap(terrain, info, tmpdir)

            assert result["min_elevation"] == pytest.approx(-5.0)
            assert result["max_elevation"] == pytest.approx(10.0)
            assert result["size_z"] == pytest.approx(15.0)
            assert result["pos_z"] == pytest.approx(-5.0)

    def test_non_square_raises(self):
        """Non-square input should raise AssertionError."""
        terrain = np.zeros((17, 33))
        info = {"size_x": 100.0, "size_y": 100.0, "grid_size": 17}

        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(ValueError):
                terrain_to_heightmap(terrain, info, tmpdir)

    def test_invalid_grid_size_raises(self):
        """Grid size not matching 2^n+1 should raise AssertionError."""
        terrain = np.zeros((10, 10))  # 10 is not 2^n + 1
        info = {"size_x": 100.0, "size_y": 100.0, "grid_size": 10}

        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(ValueError):
                terrain_to_heightmap(terrain, info, tmpdir)

    def test_handles_nan(self):
        """NaN values in terrain should be handled gracefully."""
        terrain = np.full((5, 5), -10.0)  # 2^2 + 1
        terrain[2, 2] = np.nan
        info = {"size_x": 100.0, "size_y": 100.0, "grid_size": 5}

        with tempfile.TemporaryDirectory() as tmpdir:
            result = terrain_to_heightmap(terrain, info, tmpdir)
            assert os.path.exists(result["heightmap_path"])
