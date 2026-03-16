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

"""Tests for world_builder module."""

import os
import tempfile
import xml.etree.ElementTree as ET

import pytest

from marine_charts_to_gazebo_world.world_builder import (
    _build_wave_sdf,
    generate_world_sdf,
)


class TestGenerateWorldSdf:
    def _generate(self, tmpdir, world_name="test_world"):
        """Helper to generate a world SDF and return parsed XML."""
        heightmap_info = {
            "model_name": "test_world_terrain",
            "size_x": 1000.0,
            "size_y": 1000.0,
            "size_z": 50.0,
            "pos_z": -30.0,
            "min_elevation": -30.0,
            "max_elevation": 20.0,
        }
        sdf_path = generate_world_sdf(
            world_name=world_name,
            center_lat=43.075,
            center_lon=-70.71,
            output_dir=tmpdir,
            heightmap_info=heightmap_info,
        )
        return sdf_path, ET.parse(sdf_path)

    def test_file_created(self):
        """SDF file should be created at the expected path."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path, _ = self._generate(tmpdir)
            assert os.path.exists(sdf_path)
            assert sdf_path.endswith("test_world.sdf")

    def test_valid_xml(self):
        """Generated SDF should be valid XML."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            root = tree.getroot()
            assert root.tag == "sdf"

    def test_world_name(self):
        """World element should have the correct name."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir, "portsmouth")
            world = tree.find(".//world")
            assert world is not None
            assert world.get("name") == "portsmouth"

    def test_spherical_coordinates(self):
        """Spherical coordinates should be set to the region center."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            sc = tree.find(".//spherical_coordinates")
            assert sc is not None
            lat = float(sc.findtext("latitude_deg"))
            lon = float(sc.findtext("longitude_deg"))
            assert lat == pytest.approx(43.075, abs=0.001)
            assert lon == pytest.approx(-70.71, abs=0.001)
            assert sc.findtext("surface_model") == "EARTH_WGS84"
            assert sc.findtext("world_frame_orientation") == "ENU"

    def test_physics(self):
        """DART physics with 4ms step should be configured."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            physics = tree.find(".//physics")
            assert physics is not None
            assert physics.get("type") == "dart"
            step = float(physics.findtext("max_step_size"))
            assert step == pytest.approx(0.004)

    def test_system_plugins(self):
        """Required system plugins should be present."""
        required_plugins = [
            "gz::sim::systems::Physics",
            "gz::sim::systems::Sensors",
            "gz::sim::systems::SceneBroadcaster",
            "gz::sim::systems::NavSat",
            "gz::sim::systems::Imu",
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            world = tree.find(".//world")
            plugin_names = [
                p.get("name") for p in world.findall("plugin")
            ]
            for name in required_plugins:
                assert name in plugin_names, f"Missing plugin: {name}"

    def test_terrain_include(self):
        """World should include the terrain model."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            includes = tree.findall(".//include")
            uris = [inc.findtext("uri") for inc in includes]
            assert "test_world_terrain" in uris

    def test_water_plane(self):
        """World should include a water surface plane at z=0."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            water = tree.find(".//model[@name='water_plane']")
            assert water is not None, "water_plane model not found"
            static = water.findtext("static")
            assert static == "true"

    def test_scene_configured(self):
        """Scene should have sky and grid disabled."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path, _ = self._generate(tmpdir)
            # Check raw content since <sky></sky> parses as empty element
            with open(sdf_path) as f:
                content = f.read()
            assert "<sky>" in content
            assert "<grid>false</grid>" in content

    def test_feature_models_default_empty(self):
        """Default feature_models should produce no extra models."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path, _ = self._generate(tmpdir)
            with open(sdf_path) as f:
                content = f.read()
            # Should not contain any feature model names
            assert "building_" not in content
            assert "buoy_" not in content

    def test_feature_models_included(self):
        """Feature models string should appear in the generated SDF."""
        with tempfile.TemporaryDirectory() as tmpdir:
            heightmap_info = {
                "size_x": 1000.0,
                "size_y": 1000.0,
                "size_z": 50.0,
                "pos_z": -30.0,
                "min_elevation": -30.0,
                "max_elevation": 20.0,
            }
            feature_xml = (
                '    <model name="test_buoy">'
                '<static>true</static></model>'
            )
            sdf_path = generate_world_sdf(
                world_name="test_features",
                center_lat=43.075,
                center_lon=-70.71,
                output_dir=tmpdir,
                heightmap_info=heightmap_info,
                s57_feature_models=feature_xml,
            )
            with open(sdf_path) as f:
                content = f.read()
            assert "test_buoy" in content
            # Verify it's valid XML
            tree = ET.parse(sdf_path)
            root = tree.getroot()
            assert root.tag == "sdf"


class TestWaveSdf:
    """Tests for wave/buoyancy SDF generation."""

    _HEIGHTMAP = {
        "model_name": "test_terrain",
        "size_x": 1000.0,
        "size_y": 1000.0,
        "size_z": 50.0,
        "pos_z": -30.0,
        "min_elevation": -30.0,
        "max_elevation": 20.0,
    }

    def test_no_waves_by_default(self):
        """Without wave_config, no wave content should appear."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path = generate_world_sdf(
                world_name="no_waves",
                center_lat=43.075,
                center_lon=-70.71,
                output_dir=tmpdir,
                heightmap_info=self._HEIGHTMAP,
            )
            with open(sdf_path) as f:
                content = f.read()
            assert "coast_waves" not in content
            assert "PublisherPlugin" not in content

    def test_waves_with_defaults(self):
        """wave_config={} should produce coast_waves and PublisherPlugin."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path = generate_world_sdf(
                world_name="with_waves",
                center_lat=43.075,
                center_lon=-70.71,
                output_dir=tmpdir,
                heightmap_info=self._HEIGHTMAP,
                wave_config={},
            )
            with open(sdf_path) as f:
                content = f.read()
            assert "coast_waves" in content
            assert "vrx::PublisherPlugin" in content
            assert "/vrx/wavefield/parameters" in content
            # Default values
            assert 'double_value: 0.3' in content  # gain
            assert 'double_value: 5.0' in content  # period

    def test_waves_custom_parameters(self):
        """Custom wave parameters should appear in generated SDF."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path = generate_world_sdf(
                world_name="custom_waves",
                center_lat=43.075,
                center_lon=-70.71,
                output_dir=tmpdir,
                heightmap_info=self._HEIGHTMAP,
                wave_config={"gain": 0.5, "period": 8.0, "direction": 240.0},
            )
            with open(sdf_path) as f:
                content = f.read()
            assert 'double_value: 0.5' in content
            assert 'double_value: 8.0' in content
            assert 'double_value: 240.0' in content

    def test_build_wave_sdf_contains_all_params(self):
        """_build_wave_sdf should include all four wavefield parameters."""
        sdf = _build_wave_sdf({})
        for key in ("direction", "gain", "period", "steepness"):
            assert f'key: "{key}"' in sdf

    def test_build_wave_sdf_custom_topic(self):
        """Custom topic should be used in the PublisherPlugin."""
        sdf = _build_wave_sdf({"topic": "/custom/waves"})
        assert "/custom/waves" in sdf
        assert "/vrx/wavefield/parameters" not in sdf

    def test_waves_valid_xml(self):
        """World with waves should still be valid XML."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path = generate_world_sdf(
                world_name="waves_xml",
                center_lat=43.075,
                center_lon=-70.71,
                output_dir=tmpdir,
                heightmap_info=self._HEIGHTMAP,
                wave_config={"gain": 0.7},
            )
            tree = ET.parse(sdf_path)
            assert tree.getroot().tag == "sdf"
