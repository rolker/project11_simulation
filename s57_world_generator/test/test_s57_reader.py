"""Tests for s57_reader module."""

import pytest
from osgeo import ogr

from s57_world_generator.s57_reader import (
    BoundingBox,
    _clip_geometry,
    _extract_soundings_from_multipoint,
)


class TestBoundingBox:
    def test_center(self):
        bbox = BoundingBox(south=43.0, west=-71.0, north=44.0, east=-70.0)
        assert bbox.center_lat == pytest.approx(43.5)
        assert bbox.center_lon == pytest.approx(-70.5)


class TestClipGeometry:
    def test_polygon_inside_bbox(self):
        """Polygon fully inside bbox should be returned unchanged."""
        bbox = BoundingBox(south=0.0, west=0.0, north=10.0, east=10.0)
        ring = ogr.Geometry(ogr.wkbLinearRing)
        ring.AddPoint(2.0, 2.0)
        ring.AddPoint(8.0, 2.0)
        ring.AddPoint(8.0, 8.0)
        ring.AddPoint(2.0, 8.0)
        ring.AddPoint(2.0, 2.0)
        poly = ogr.Geometry(ogr.wkbPolygon)
        poly.AddGeometry(ring)

        result = _clip_geometry(poly, bbox)
        assert result is not None
        assert result.GetArea() == pytest.approx(36.0, abs=0.01)

    def test_polygon_outside_bbox(self):
        """Polygon fully outside bbox should return None."""
        bbox = BoundingBox(south=0.0, west=0.0, north=10.0, east=10.0)
        ring = ogr.Geometry(ogr.wkbLinearRing)
        ring.AddPoint(20.0, 20.0)
        ring.AddPoint(30.0, 20.0)
        ring.AddPoint(30.0, 30.0)
        ring.AddPoint(20.0, 30.0)
        ring.AddPoint(20.0, 20.0)
        poly = ogr.Geometry(ogr.wkbPolygon)
        poly.AddGeometry(ring)

        result = _clip_geometry(poly, bbox)
        assert result is None

    def test_polygon_partially_inside_bbox(self):
        """Polygon crossing bbox boundary should be clipped."""
        bbox = BoundingBox(south=0.0, west=0.0, north=10.0, east=10.0)
        ring = ogr.Geometry(ogr.wkbLinearRing)
        ring.AddPoint(5.0, 5.0)
        ring.AddPoint(15.0, 5.0)
        ring.AddPoint(15.0, 15.0)
        ring.AddPoint(5.0, 15.0)
        ring.AddPoint(5.0, 5.0)
        poly = ogr.Geometry(ogr.wkbPolygon)
        poly.AddGeometry(ring)

        result = _clip_geometry(poly, bbox)
        assert result is not None
        # Clipped to 5x5 area
        assert result.GetArea() == pytest.approx(25.0, abs=0.01)


class TestExtractSoundings:
    def test_single_point(self):
        """Extract depth from a single 3D point."""
        bbox = BoundingBox(south=42.0, west=-71.0, north=44.0, east=-69.0)
        pt = ogr.Geometry(ogr.wkbPoint25D)
        pt.AddPoint(-70.0, 43.0, 15.5)

        result = _extract_soundings_from_multipoint(pt, bbox)
        assert len(result) == 1
        assert result[0].lat == pytest.approx(43.0)
        assert result[0].lon == pytest.approx(-70.0)
        assert result[0].depth == pytest.approx(15.5)

    def test_multipoint(self):
        """Extract depths from a multipoint geometry."""
        bbox = BoundingBox(south=42.0, west=-71.0, north=44.0, east=-69.0)
        mp = ogr.Geometry(ogr.wkbMultiPoint25D)

        pt1 = ogr.Geometry(ogr.wkbPoint25D)
        pt1.AddPoint(-70.5, 43.0, 10.0)
        mp.AddGeometry(pt1)

        pt2 = ogr.Geometry(ogr.wkbPoint25D)
        pt2.AddPoint(-70.0, 43.5, 20.0)
        mp.AddGeometry(pt2)

        result = _extract_soundings_from_multipoint(mp, bbox)
        assert len(result) == 2

    def test_point_outside_bbox(self):
        """Points outside bbox should be excluded."""
        bbox = BoundingBox(south=42.0, west=-71.0, north=44.0, east=-69.0)
        pt = ogr.Geometry(ogr.wkbPoint25D)
        pt.AddPoint(-75.0, 43.0, 15.5)  # lon outside bbox

        result = _extract_soundings_from_multipoint(pt, bbox)
        assert len(result) == 0
