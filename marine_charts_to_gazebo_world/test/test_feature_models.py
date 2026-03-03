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

"""Tests for feature_models module."""

import xml.etree.ElementTree as ET

import pytest
from osgeo import ogr

from marine_charts_to_gazebo_world.feature_models import (
    _latlon_to_enu,
    generate_feature_models,
)
from marine_charts_to_gazebo_world.s57_reader import (
    Beacon,
    Bridge,
    Building,
    Buoy,
    Light,
    Pontoon,
    S57Features,
)

CENTER_LAT = 43.075
CENTER_LON = -70.71


def _make_polygon(coords):
    """Create an OGR polygon from a list of (lon, lat) tuples."""
    ring = ogr.Geometry(ogr.wkbLinearRing)
    for lon, lat in coords:
        ring.AddPoint(lon, lat)
    ring.AddPoint(coords[0][0], coords[0][1])  # close ring
    poly = ogr.Geometry(ogr.wkbPolygon)
    poly.AddGeometry(ring)
    return poly


class TestLatLonToEnu:

    def test_center_is_origin(self):
        """World center should map to (0, 0)."""
        x, y = _latlon_to_enu(CENTER_LAT, CENTER_LON, CENTER_LAT, CENTER_LON)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)

    def test_north_offset(self):
        """Point north of center should have positive y offset."""
        x, y = _latlon_to_enu(
            CENTER_LAT + 0.001, CENTER_LON, CENTER_LAT, CENTER_LON
        )
        assert x == pytest.approx(0.0, abs=0.01)
        assert y > 0
        assert y == pytest.approx(111.32, abs=1.0)

    def test_east_offset(self):
        """Point east of center should have positive x offset."""
        x, y = _latlon_to_enu(
            CENTER_LAT, CENTER_LON + 0.001, CENTER_LAT, CENTER_LON
        )
        assert y == pytest.approx(0.0, abs=0.01)
        assert x > 0


class TestGenerateFeatureModelsEmpty:

    def test_empty_features(self):
        """Empty features should return empty string."""
        features = S57Features()
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert result == ''


class TestBuildingModel:

    def test_building_generates_polyline(self):
        """Building polygon should produce valid SDF with <polyline>."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(buildings=[Building(geometry=poly, objl=12)])
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="building_0000">' in result
        assert '<polyline>' in result
        assert '<height>5.0</height>' in result
        # Should be valid XML when wrapped
        root = ET.fromstring(f'<root>{result}</root>')
        models = root.findall('model')
        assert len(models) == 1

    def test_landmark_height(self):
        """LNDMRK (OBJL 73) should use 12.0m height."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(buildings=[Building(geometry=poly, objl=73)])
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<height>12.0</height>' in result

    def test_silo_height(self):
        """SILTNK (OBJL 119) should use 8.0m height."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(buildings=[Building(geometry=poly, objl=119)])
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<height>8.0</height>' in result


class TestPontoonModel:

    def test_pontoon_generates_model(self):
        """Pontoon polygon should produce SDF with correct height."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(pontoons=[Pontoon(geometry=poly)])
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="pontoon_0000">' in result
        assert '<height>1.0</height>' in result


class TestBridgeModel:

    def test_bridge_with_clearance(self):
        """Bridge with VERCLR should use clearance as height."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(
            bridges=[Bridge(geometry=poly, clearance=15.0)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="bridge_0000">' in result
        assert '<height>15.0</height>' in result

    def test_bridge_default_height(self):
        """Bridge without clearance should use default 10.0m."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(bridges=[Bridge(geometry=poly, clearance=0.0)])
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<height>10.0</height>' in result


class TestBuoyModel:

    def test_buoy_generates_cylinder_and_cone(self):
        """Buoy should have cylinder body and cone top."""
        features = S57Features(
            buoys=[Buoy(lat=43.076, lon=-70.711, colour=3)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="buoy_0000">' in result
        assert '<cylinder>' in result
        assert '<cone>' in result
        root = ET.fromstring(f'<root>{result}</root>')
        assert len(root.findall('model')) == 1

    def test_buoy_red_colour(self):
        """Buoy with COLOUR=3 (red) should have red material."""
        features = S57Features(
            buoys=[Buoy(lat=43.076, lon=-70.711, colour=3)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        # Red: 1.0 0.0 0.0
        assert '1.0 0.0 0.0 1.0' in result

    def test_buoy_green_colour(self):
        """Buoy with COLOUR=4 (green) should have green material."""
        features = S57Features(
            buoys=[Buoy(lat=43.076, lon=-70.711, colour=4)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '0.0 0.8 0.0 1.0' in result

    def test_buoy_unknown_colour_defaults_yellow(self):
        """Buoy with unknown COLOUR should default to yellow."""
        features = S57Features(
            buoys=[Buoy(lat=43.076, lon=-70.711, colour=99)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '1.0 1.0 0.0 1.0' in result


class TestBeaconModel:

    def test_beacon_generates_cylinder(self):
        """Beacon should produce a cylinder model."""
        features = S57Features(
            beacons=[Beacon(lat=43.076, lon=-70.711)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="beacon_0000">' in result
        assert '<cylinder>' in result
        root = ET.fromstring(f'<root>{result}</root>')
        assert len(root.findall('model')) == 1


class TestLightModel:

    def test_light_generates_pole_and_lamp(self):
        """Light should produce a pole cylinder and lamp sphere."""
        features = S57Features(
            lights=[Light(lat=43.076, lon=-70.711)]
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert '<model name="light_0000">' in result
        assert '<cylinder>' in result
        assert '<sphere>' in result
        root = ET.fromstring(f'<root>{result}</root>')
        assert len(root.findall('model')) == 1


class TestMultipleFeatures:

    def test_mixed_features(self):
        """Multiple feature types should all appear in output."""
        poly = _make_polygon([
            (-70.711, 43.076),
            (-70.710, 43.076),
            (-70.710, 43.077),
            (-70.711, 43.077),
        ])
        features = S57Features(
            buildings=[Building(geometry=poly, objl=12)],
            buoys=[Buoy(lat=43.076, lon=-70.711, colour=3)],
            lights=[Light(lat=43.076, lon=-70.712)],
        )
        result = generate_feature_models(features, CENTER_LAT, CENTER_LON)
        assert 'building_0000' in result
        assert 'buoy_0000' in result
        assert 'light_0000' in result
        root = ET.fromstring(f'<root>{result}</root>')
        assert len(root.findall('model')) == 3
