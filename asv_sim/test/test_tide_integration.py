"""Integration test: verify tide model publishes via ROS topics.

Launches asv_sim with accelerated tide (speed_factor=3600 compresses
a ~12-hour cycle into ~12 seconds) and verifies that:
- ~/environment/tide_level topic publishes varying values
- NavSatFix.altitude is populated and varies with tide

Run with: launch_test test/test_tide_integration.py

This file must be run via launch_test, not plain pytest.
The conftest.py collect_ignore list excludes it from colcon test.
"""

import unittest

import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node as RclpyNode
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


def generate_test_description():
    """Launch asv_sim with ben platform and accelerated tide."""
    asv_sim_node = Node(
        package='asv_sim',
        executable='asv_sim',
        name='asv_sim',
        parameters=[
            {'platforms': ['ben']},
            {'platforms.ben.model': 'cw4'},
            {'platforms.ben.namespace': '/ben'},
            {'platforms.ben.start_lat': 43.073},
            {'platforms.ben.start_lon': -70.711},
            {'platforms.ben.start_heading': 60.0},
            {'platforms.ben.mru_frame': 'ben/mru'},
            # cw4 model params (minimal set)
            {'models.cw4.max_rpm': 3200.0},
            {'models.cw4.max_power': 21300.0},
            {'models.cw4.idle_rpm': 0.0},
            {'models.cw4.max_speed': 2.75},
            {'models.cw4.mass': 2000.0},
            {'models.cw4.max_rudder_angle': 30.0},
            {'models.cw4.max_rudder_change_rate': 30.0},
            {'models.cw4.rudder_distance': 1.48},
            {'models.cw4.propulsion_type': 'jet'},
            # Accelerate tide: 3600x = 12-hour cycle in ~12 seconds
            {'environment.tide.speed_factor': 3600.0},
        ],
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        asv_sim_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTideIntegration(unittest.TestCase):
    """Tests that run while the node is alive."""

    @classmethod
    def setUpClass(cls):
        """Create a test node for subscribing to topics."""
        rclpy.init()
        cls.node = RclpyNode('test_tide_integration')
        cls.tide_values = []
        cls.altitude_values = []

        cls.tide_sub = cls.node.create_subscription(
            Float64,
            '/asv_sim/environment/tide_level',
            cls._tide_callback,
            10,
        )
        cls.position_sub = cls.node.create_subscription(
            NavSatFix,
            '/ben/position',
            cls._position_callback,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        """Shut down the test node."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _tide_callback(cls, msg):
        cls.tide_values.append(msg.data)

    @classmethod
    def _position_callback(cls, msg):
        cls.altitude_values.append(msg.altitude)

    def _collect_samples(self, seconds=5.0):
        """Spin the node collecting messages for the given duration."""
        end_time = self.node.get_clock().now() + Duration(
            seconds=seconds)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.get_clock().now() > end_time:
                break

    def test_tide_topic_publishes(self):
        """The tide_level topic should publish values."""
        self._collect_samples(5.0)
        self.assertGreater(
            len(self.tide_values), 5,
            'Expected multiple tide_level messages')

    def test_tide_varies(self):
        """With speed_factor=3600, tide should vary visibly in 5 seconds."""
        # Data collected in test_tide_topic_publishes
        if len(self.tide_values) < 2:
            self._collect_samples(5.0)
        tide_range = max(self.tide_values) - min(self.tide_values)
        self.assertGreater(
            tide_range, 0.5,
            f'Tide range {tide_range:.3f}m is too small — '
            f'expected visible variation with speed_factor=3600')

    def test_altitude_published(self):
        """NavSatFix.altitude should be populated (not zero)."""
        if len(self.altitude_values) < 2:
            self._collect_samples(5.0)
        self.assertGreater(
            len(self.altitude_values), 5,
            'Expected multiple NavSatFix messages with altitude')
        # Altitude should be around -28m (ellipsoid to MLLW offset)
        mean_alt = sum(self.altitude_values) / len(self.altitude_values)
        self.assertLess(mean_alt, -25.0,
                        f'Mean altitude {mean_alt:.1f}m should be < -25')
        self.assertGreater(mean_alt, -32.0,
                           f'Mean altitude {mean_alt:.1f}m should be > -32')

    def test_altitude_varies_with_tide(self):
        """NavSatFix.altitude should vary as tide changes."""
        if len(self.altitude_values) < 2:
            self._collect_samples(5.0)
        alt_range = max(self.altitude_values) - min(self.altitude_values)
        self.assertGreater(
            alt_range, 0.5,
            f'Altitude range {alt_range:.3f}m is too small — '
            f'expected variation from accelerated tide')


@launch_testing.post_shutdown_test()
class TestTideShutdown(unittest.TestCase):
    """Tests that run after the node is shut down."""

    def test_exit_code(self, proc_info):
        """Node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],
        )
