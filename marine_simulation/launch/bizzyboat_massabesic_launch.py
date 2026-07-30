from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the BizzyBoat (EchoBoat 240) sim at Lake Massabesic (non-Gazebo).

    One-line entry: wraps sim_robot_launch.py with platform:=bizzy. Brings up
    the boat's real autonomy + nav2 stack (via bizzyboat_sim_core_launch.py,
    reusing the field nav2_overlay.yaml), the echoboat240 asv_sim dynamics model
    spawned at the 2026-06-12 last-recorded pose with no current, and the
    simulated MBES sampling the Massabesic ground-truth grid. The boat's costmap
    starts from only its field config (currently chart_layer disabled, local
    costmap 0.5 m / 350 m — the 2026-06-12 field changes), so this reproduces the
    "planner won't plan beyond the local costmap" setup.

    Also brings up the operator station for `bizzy` (via sim_operator_launch):
    `command_bridge_sender` (so CAMP's piloting-mode commands reach helm_manager)
    and CAMP itself — no background chart, no RViz. Because this launches CAMP, do
    NOT run a separate CAMP instance alongside it. Bag recording (a
    simulator_launch.py extra) is still not included here.
    """
    mbes_grid_file = LaunchConfiguration('mbes_grid_file')
    mbes_grid_file_arg = DeclareLaunchArgument(
        'mbes_grid_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mbes_sim'), 'data', 'US5NH02M.tiff'
        ]),
        description='Massabesic ground-truth bathymetry GeoTIFF for the '
        'simulated MBES. NOTE: the default packaged sample is a Portsmouth NH '
        'grid (~100 km away), so the MBES produces NO depth/detections at the '
        'Massabesic spawn until a real Massabesic bathymetry grid is built and '
        'passed here (follow-up).'
    )

    return LaunchDescription([
        mbes_grid_file_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('marine_simulation'),
                    'launch',
                    'sim_robot_launch.py'
                ])
            ),
            launch_arguments={
                'platform': 'bizzy',
                'namespace': 'bizzy',
                'sim_name': 'bizzy',
                'enable_bridge': 'false',
                'mbes_grid_file': mbes_grid_file,
            }.items()
        ),

        # Operator station for bizzy (mirrors simulator_launch.py:103-123):
        # command_bridge_sender (robot ns) + CAMP, on the one sim ROS graph, so
        # CAMP's piloting-mode commands reach helm_manager. No udp_bridge needed
        # (single domain), no background chart, no RViz.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('marine_simulation'),
                    'launch',
                    'sim_operator_launch.py'
                ])
            ),
            launch_arguments={
                'robot_namespace': 'bizzy',
                'operator_namespace': 'operator',
                'enable_bridge': 'false',
                'rviz': 'false',
            }.items()
        ),
    ])
