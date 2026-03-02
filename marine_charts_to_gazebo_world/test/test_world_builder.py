"""Tests for world_builder module."""

import os
import tempfile
import xml.etree.ElementTree as ET

import pytest

from marine_charts_to_gazebo_world.world_builder import generate_world_sdf


class TestGenerateWorldSdf:
    def _generate(self, tmpdir, world_name="test_world"):
        """Helper to generate a world SDF and return parsed XML."""
        heightmap_info = {
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
            assert "models/terrain" in uris

    def test_coast_waves_include(self):
        """World should include VRX coast_waves model."""
        with tempfile.TemporaryDirectory() as tmpdir:
            _, tree = self._generate(tmpdir)
            includes = tree.findall(".//include")
            found = any(
                inc.findtext("uri") == "coast_waves"
                for inc in includes
            )
            assert found, "coast_waves model not found in includes"

    def test_scene_configured(self):
        """Scene should have sky and grid disabled."""
        with tempfile.TemporaryDirectory() as tmpdir:
            sdf_path, _ = self._generate(tmpdir)
            # Check raw content since <sky></sky> parses as empty element
            with open(sdf_path) as f:
                content = f.read()
            assert "<sky>" in content
            assert "<grid>false</grid>" in content
