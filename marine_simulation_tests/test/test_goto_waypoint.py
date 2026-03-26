# Copyright 2026 University of New Hampshire
# SPDX-License-Identifier: BSD-3-Clause

"""End-to-end test: goto single waypoint.

Launches the full simulation stack (asv_sim, asv_helm, nav2,
mission_manager, helm_manager) and commands the vehicle to navigate
to a waypoint ~100m from the start position. Verifies arrival by
monitoring the NavSatFix position.

Test tier: Minimal (target <2 min).
"""

import json
import unittest

import launch
import launch_testing
import launch_testing.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare

from marine_simulation_tests.simulation_test_base import SimulationTestBase
import pytest


# Target: ~100m from start at heading ~60deg
TARGET_LAT = 43.07384
TARGET_LON = -70.70983


@pytest.mark.rostest
def generate_test_description():
    """Launch the full simulation robot stack with test BT XML."""
    # Override the default BT XML to one that doesn't require
    # compute_sonar_coverage_path action server.
    test_bt_xml = PathJoinSubstitution([
        FindPackageShare('marine_simulation_tests'),
        'config',
        'run_tasks_test.xml',
    ])

    nav2_test_params = PathJoinSubstitution([
        FindPackageShare('marine_simulation_tests'),
        'config',
        'nav2_test_params.yaml',
    ])

    sim_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('marine_simulation'),
                'launch',
                'sim_robot_launch.py',
            ])
        ),
        launch_arguments={
            'namespace': 'ben',
            'enable_bridge': 'false',
            'params_file': nav2_test_params,
        }.items(),
    )

    return (
        launch.LaunchDescription([
            # Use test BT XML (no sonar coverage dependency).
            SetParameter(
                name='default_task_navigator_bt_xml',
                value=test_bt_xml,
            ),
            sim_robot_launch,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestGotoWaypoint(SimulationTestBase):
    """Test that the vehicle can navigate to a single waypoint."""

    def test_goto_waypoint(self):
        """Command goto waypoint and verify arrival.

        Steps:
        1. Wait for simulation startup (NavSatFix received)
        2. Wait for nav2 stack to be ready
        3. Set autonomous piloting mode
        4. Send goto waypoint mission command
        5. Wait for Navigator=done with goto_0 completed
        6. Verify vehicle position is within tolerance of target
        """
        # 1. Wait for simulation to start producing position data.
        self.wait_for_position(timeout=30.0)

        # 2. Wait for nav2 lifecycle activation to complete.
        self.wait_for_nav2_ready(timeout=30.0)

        # 3. Set autonomous mode so helm_manager accepts nav commands.
        self.set_autonomous_mode()

        # 4. Send goto waypoint command.
        mission = json.dumps([{
            'type': 'Waypoint',
            'latitude': TARGET_LAT,
            'longitude': TARGET_LON,
        }])
        self.send_command(f'append_task mission_plan {mission}')

        # 5. Wait for goto_0 task completion.
        # The navigator stays running after goto completes (done_hover
        # task keeps it busy), so check task status directly rather
        # than waiting for Navigator=done.
        done = self.wait_for_task_done('goto_0', timeout=90.0)
        self.assertTrue(
            done,
            'goto_0 was not marked done within 90s. '
            'The vehicle may not have reached the waypoint.',
        )

        # 6. Verify position.
        self.assert_arrived(TARGET_LAT, TARGET_LON, tolerance_m=10.0)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Verify all processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        """Check that launched processes exited without crashing."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15]
        )
