"""Tests for the tidal model in asv_sim.environment."""

import math

from asv_sim.environment import Environment
import pytest
import rclpy
import rclpy.time


@pytest.fixture(scope='module')
def node():
    """Create a ROS node for the test module."""
    rclpy.init()
    n = rclpy.node.Node('test_tide')
    yield n
    n.destroy_node()
    rclpy.shutdown()


@pytest.fixture(scope='module')
def env(node):
    """Create a single Environment instance for all tests."""
    return Environment(node)


def _make_time(hours_since_epoch: float) -> rclpy.time.Time:
    """Create a ROS Time from hours since epoch."""
    nanoseconds = int(hours_since_epoch * 3.6e12)
    return rclpy.time.Time(nanoseconds=nanoseconds)


class TestTideModel:
    """Test harmonic tide computation."""

    def test_tide_returns_float(self, env):
        """Verify getTide returns a float."""
        t = _make_time(0.0)
        result = env.getTide(t)
        assert isinstance(result, float)

    def test_tide_amplitude_range(self, env):
        """Verify tide stays within Z0 +/- sum-of-amplitudes bounds."""
        max_amplitude = 1.295 + 0.289 + 0.197  # sum of default amplitudes
        z0 = 1.43  # MSL above MLLW
        # Sample tide over a full M2 cycle (~12.42 hours)
        values = []
        for minutes in range(0, 750, 1):  # 12.5 hours in 1-minute steps
            t = _make_time(minutes / 60.0)
            values.append(env.getTide(t))

        assert max(values) <= z0 + max_amplitude + 0.001
        assert min(values) >= z0 - max_amplitude - 0.001
        # Most values should be positive (above MLLW); extreme lows
        # can dip slightly below since MLLW is a mean, not a minimum
        assert min(values) > -0.5
        # Should have significant variation
        assert max(values) - min(values) > 1.0

    def test_tide_varies_over_time(self, env):
        """Tide should not be constant."""
        t1 = _make_time(0.0)
        t2 = _make_time(3.0)  # 3 hours later
        t3 = _make_time(6.0)  # 6 hours later
        v1 = env.getTide(t1)
        v2 = env.getTide(t2)
        v3 = env.getTide(t3)
        # At least two of these should differ
        assert not (v1 == v2 == v3)

    def test_ellipsoidal_altitude_includes_offset(self, env):
        """Ellipsoidal altitude equals ellipsoid_to_mllw plus tide."""
        t = _make_time(0.0)
        tide = env.getTide(t)
        altitude = env.getEllipsoidalAltitude(t)
        expected = env.ellipsoid_to_mllw + tide
        assert abs(altitude - expected) < 1e-10

    def test_ellipsoidal_altitude_negative(self, env):
        """Altitude should be strongly negative at Portsmouth (~-28m)."""
        t = _make_time(0.0)
        altitude = env.getEllipsoidalAltitude(t)
        assert altitude < -25.0
        assert altitude > -32.0

    def test_speed_factor_effect(self, env):
        """Doubling speed should produce different tide at same time."""
        t = _make_time(6.0)
        normal = env.getTide(t)

        # Compute what tide would be with doubled speed manually
        hours = 6.0
        accelerated = env.msl_above_mllw
        for amp, spd, pha in zip(
            env.tide_amplitudes,
            env.tide_speeds,
            env.tide_phases,
        ):
            speed_rad = math.radians(spd * 2.0)
            phase_rad = math.radians(pha)
            accelerated += amp * math.cos(speed_rad * hours - phase_rad)

        # These should differ (different effective speeds)
        assert abs(normal - accelerated) > 0.01
