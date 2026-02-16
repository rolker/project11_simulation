# Copyright 2026 University of New Hampshire
# SPDX-License-Identifier: BSD-3-Clause

"""Shared helpers for end-to-end simulation tests.

Provides a base class with ROS node setup, position monitoring,
mission command helpers, and haversine distance calculation.
"""

import math
import time
import unittest

import rclpy
from marine_interfaces.msg import Heartbeat
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class SimulationTestBase(unittest.TestCase):
    """Base class for simulation end-to-end tests.

    Subclasses get a ROS node with publishers and subscribers for
    piloting mode, mission commands, position, and mission status.
    """

    NAMESPACE = 'ben'
    START_LAT = 43.073397415457535
    START_LON = -70.71054174878898

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node, publishers, subscribers, and reset state."""
        self.node = rclpy.create_node('simulation_test_node')

        self.piloting_mode_pub = self.node.create_publisher(
            String, f'/{self.NAMESPACE}/piloting_mode', 10)

        self.cmd_pub = self.node.create_publisher(
            String,
            f'/{self.NAMESPACE}/marine/mission_manager/command',
            10,
        )

        self.latest_position = None
        self.position_sub = self.node.create_subscription(
            NavSatFix,
            f'/{self.NAMESPACE}/sensors/nav/position',
            self._position_callback,
            10,
        )

        self.heartbeats_rx = []
        self.heartbeat_sub = self.node.create_subscription(
            Heartbeat,
            f'/{self.NAMESPACE}/marine/status/mission_manager',
            lambda msg: self.heartbeats_rx.append(msg),
            10,
        )

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    # -- Callbacks --

    def _position_callback(self, msg):
        """Store latest NavSatFix."""
        self.latest_position = msg

    # -- Startup helpers --

    def wait_for_position(self, timeout=30.0):
        """Spin until the first NavSatFix is received.

        This indicates the simulation is running and publishing sensor
        data, meaning asv_sim + ben_core are up.
        """
        ok = self._spin_until(
            lambda: self.latest_position is not None,
            timeout,
        )
        self.assertTrue(
            ok,
            f'No NavSatFix received on /{self.NAMESPACE}/sensors/nav/'
            f'position within {timeout}s — simulation may not have '
            'started.',
        )

    def wait_for_nav2_ready(self, timeout=30.0):
        """Wait for the nav2 stack to be fully active.

        The bt_task_navigator needs time for lifecycle activation.
        We wait for any heartbeat from mission_manager that contains
        a Navigator key, which indicates bt_navigator accepted a goal
        and the nav2 stack is operational. If no Navigator heartbeat
        arrives within the first 15s, we send clear_tasks to trigger
        a new goal.
        """
        def _has_navigator_heartbeat():
            return any(
                kv.key == 'Navigator'
                for hb in self.heartbeats_rx
                for kv in hb.values
            )

        # First, wait for heartbeats to start flowing.
        got_nav = self._spin_until(
            _has_navigator_heartbeat,
            timeout=min(timeout, 15.0),
        )
        if not got_nav:
            # Nav2 may have activated after the initial goal was
            # rejected. Send clear_tasks to trigger updateNavigator()
            # which re-sends the done_hover goal.
            self.node.get_logger().info(
                'No initial Navigator heartbeat. '
                'Sending clear_tasks to re-trigger navigation goal.')
            msg = String(data='clear_tasks')
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

            got_nav = self._spin_until(
                _has_navigator_heartbeat,
                timeout=max(timeout - 15.0, 10.0),
            )

        self.assertTrue(
            got_nav,
            f'Nav2 stack did not become ready within {timeout}s. '
            'No Navigator heartbeat received.',
        )
        # Clear collected heartbeats so tests start fresh.
        self.heartbeats_rx.clear()

    def set_autonomous_mode(self):
        """Publish 'autonomous' to piloting_mode.

        Publishes multiple times over 1s to ensure helm_manager
        receives the mode change even during startup discovery.
        """
        msg = String(data='autonomous')
        end_time = time.time() + 1.0
        while time.time() < end_time:
            self.piloting_mode_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # -- Command helpers --

    def send_command(self, cmd):
        """Publish a single command string to mission_manager.

        Waits for at least one subscriber on the command topic before
        publishing, to ensure mission_manager is connected. Publishes
        exactly once to avoid duplicate task appends.
        """
        # Wait for mission_manager to subscribe to our command topic.
        ok = self._spin_until(
            lambda: self.cmd_pub.get_subscription_count() > 0,
            timeout=10.0,
        )
        self.assertTrue(
            ok,
            'No subscriber found on mission_manager/command within 10s.',
        )
        msg = String(data=cmd)
        self.cmd_pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.2)

    # -- Spin / wait helpers --

    def _spin_until(self, predicate, timeout=20.0):
        """Spin the node until predicate returns True or timeout."""
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def wait_for_navigator_done(self, timeout=90.0):
        """Wait for a heartbeat with Navigator=done.

        Returns True if a Navigator=done heartbeat was received.
        """
        return self._spin_until(
            lambda: self._has_heartbeat_kv('Navigator', 'done'),
            timeout,
        )

    def wait_for_mission_done(self, task_id, timeout=90.0):
        """Wait for Navigator=done heartbeat where task_id is marked done.

        When a new goal preempts a running one, a Navigator=done
        heartbeat is emitted for the preempted goal (with tasks not
        yet done). This method waits specifically for a done heartbeat
        where the given task is marked as completed.
        """
        def _check():
            for hb in self.heartbeats_rx:
                is_done = False
                task_done = False
                for kv in hb.values:
                    if kv.key == 'Navigator' and kv.value == 'done':
                        is_done = True
                    if kv.key == task_id and '(done)' in kv.value:
                        task_done = True
                if is_done and task_done:
                    return True
            return False
        return self._spin_until(_check, timeout)

    # -- Heartbeat helpers --

    def _has_heartbeat_kv(self, key, value):
        """Check if any collected heartbeat contains a KV pair."""
        return any(
            kv.key == key and kv.value == value
            for hb in self.heartbeats_rx
            for kv in hb.values
        )

    def last_done_heartbeat(self):
        """Return the most recent heartbeat with Navigator=done."""
        for hb in reversed(self.heartbeats_rx):
            for kv in hb.values:
                if kv.key == 'Navigator' and kv.value == 'done':
                    return hb
        return None

    def heartbeat_task_value(self, hb, task_id):
        """Get the value string for a task_id in a heartbeat, or None."""
        if hb is None:
            return None
        for kv in hb.values:
            if kv.key == task_id:
                return kv.value
        return None

    # -- Position / distance helpers --

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        """Compute great-circle distance in meters between two points."""
        R = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = (math.sin(dphi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def assert_arrived(self, target_lat, target_lon, tolerance_m=10.0):
        """Assert the vessel is within tolerance of the target position."""
        self.assertIsNotNone(
            self.latest_position,
            'No position data available to check arrival.',
        )
        dist = self.haversine_distance(
            self.latest_position.latitude,
            self.latest_position.longitude,
            target_lat, target_lon,
        )
        self.assertLessEqual(
            dist, tolerance_m,
            f'Vessel at ({self.latest_position.latitude:.6f}, '
            f'{self.latest_position.longitude:.6f}) is {dist:.1f}m from '
            f'target ({target_lat:.6f}, {target_lon:.6f}), '
            f'exceeds {tolerance_m}m tolerance.',
        )

    def assert_hover_stable(self, lat, lon, tolerance_m=25.0, duration=10.0):
        """Assert the vessel stays within tolerance for duration seconds.

        Spins the node and checks position repeatedly. Fails if the
        vessel ever exceeds tolerance from the given position.
        """
        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.latest_position is not None:
                dist = self.haversine_distance(
                    self.latest_position.latitude,
                    self.latest_position.longitude,
                    lat, lon,
                )
                self.assertLessEqual(
                    dist, tolerance_m,
                    f'Vessel drifted {dist:.1f}m from hover position '
                    f'({lat:.6f}, {lon:.6f}) during stability check '
                    f'(tolerance {tolerance_m}m).',
                )
