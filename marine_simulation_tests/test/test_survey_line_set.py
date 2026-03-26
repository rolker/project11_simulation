# Copyright 2026 University of New Hampshire
# SPDX-License-Identifier: BSD-3-Clause

"""End-to-end test: survey line set with hover.

Launches the full simulation stack and commands the vehicle to execute
a 2-line survey pattern. Verifies completion via mission_manager
heartbeats and checks that the vehicle hovers near the final waypoint
after the survey completes.

Test tier: Medium (target <5 min).
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


# Two survey lines near the start position.
# Line 1: SW to NE (~80m)
LINE_1_START = {'latitude': 43.0735, 'longitude': -70.7100}
LINE_1_END = {'latitude': 43.0739, 'longitude': -70.7095}
# Line 2: NE to SW (~80m, reciprocal)
LINE_2_START = {'latitude': 43.0739, 'longitude': -70.7093}
LINE_2_END = {'latitude': 43.0735, 'longitude': -70.7098}

# Final position should be near end of line 2.
FINAL_LAT = LINE_2_END['latitude']
FINAL_LON = LINE_2_END['longitude']


@pytest.mark.rostest
def generate_test_description():
    """Launch the full simulation robot stack with test BT XML."""
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


class TestSurveyLineSet(SimulationTestBase):
    """Test that the vehicle can execute a 2-line survey and hover."""

    def test_survey_line_set(self):
        """Command a 2-line survey and verify completion + hover.

        Steps:
        1. Wait for simulation startup (NavSatFix received)
        2. Wait for nav2 stack to be ready
        3. Set autonomous piloting mode
        4. Send survey line set mission command
        5. Wait for Navigator=done heartbeat
        6. Verify all tasks show done in final heartbeat
        7. Verify vessel hovers near final waypoint
        """
        # 1. Wait for simulation to start.
        self.wait_for_position(timeout=30.0)

        # 2. Wait for nav2 lifecycle activation to complete.
        self.wait_for_nav2_ready(timeout=30.0)

        # 3. Set autonomous mode.
        self.set_autonomous_mode()

        # 4. Build and send survey mission.
        mission = json.dumps([{
            'type': 'SurveyPattern',
            'label': 'test_survey',
            'children': [
                {
                    'type': 'TrackLine',
                    'label': 'line_1',
                    'children': [
                        {'type': 'Waypoint', **LINE_1_START},
                        {'type': 'Waypoint', **LINE_1_END},
                    ],
                },
                {
                    'type': 'TrackLine',
                    'label': 'line_2',
                    'children': [
                        {'type': 'Waypoint', **LINE_2_START},
                        {'type': 'Waypoint', **LINE_2_END},
                    ],
                },
            ],
        }])
        self.send_command(f'append_task mission_plan {mission}')

        # 5. Wait for survey task completion (longer timeout for multi-line).
        # The navigator stays running after survey completes (done_hover
        # task keeps it busy), so check task status directly.
        done = self.wait_for_task_done('test_survey', timeout=180.0)
        self.assertTrue(
            done,
            'test_survey was not marked done within 180s. '
            'The survey may not have completed.',
        )

        # 6. Verify task completion in latest heartbeat.
        # Find the most recent heartbeat with test_survey marked done.
        done_hb = self.last_heartbeat_with_task_done('test_survey')
        self.assertIsNotNone(
            done_hb,
            'No heartbeat with test_survey (done) found.',
        )

        # 7. Verify hover stability near final waypoint.
        self.assert_arrived(FINAL_LAT, FINAL_LON, tolerance_m=25.0)
        self.assert_hover_stable(
            FINAL_LAT, FINAL_LON,
            tolerance_m=25.0,
            duration=10.0,
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Verify all processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        """Check that launched processes exited without crashing."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15]
        )
