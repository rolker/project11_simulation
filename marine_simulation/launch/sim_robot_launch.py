# Copyright 2026 Roland Arsenault, UNH CCOM
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sim_name = LaunchConfiguration('sim_name')
    enable_bridge = LaunchConfiguration('enable_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drix = LaunchConfiguration('drix')
    platform = LaunchConfiguration('platform')
    no_sim = LaunchConfiguration('no_sim')
    tide_speed_factor = LaunchConfiguration('tide_speed_factor')
    mbes_grid_file = LaunchConfiguration('mbes_grid_file')

    # Platform selector. 'ben' (default) brings up ben_core_launch + the cw4
    # model; 'bizzy' brings up bizzyboat_sim_core_launch + the echoboat240
    # model (BizzyBoat at Lake Massabesic). The legacy `drix` flag is preserved
    # for the (currently commented) DRIX path, so is_ben also gates on it.
    is_ben = IfCondition(PythonExpression(
        ["'", platform, "' == 'ben' and '", drix, "' == 'false'"]))
    is_bizzy = IfCondition(PythonExpression(["'", platform, "' == 'bizzy'"]))

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='ben')
    )
    platform_arg = DeclareLaunchArgument(
        'platform', default_value=TextSubstitution(text='ben'),
        description="Simulated platform: 'ben' or 'bizzy'."
    )
    mbes_grid_file_arg = DeclareLaunchArgument(
        'mbes_grid_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('mbes_sim'), 'data', 'US5NH02M.tiff'
        ]),
        description='Ground-truth bathymetry GeoTIFF for the simulated MBES. '
        'For BizzyBoat-Massabesic, point this at the Massabesic bathymetry grid.'
    )
    sim_name_arg = DeclareLaunchArgument(
        'sim_name', default_value=namespace
    )
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge', default_value=TextSubstitution(text='true')
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value=TextSubstitution(text='false')
    )
    drix_arg = DeclareLaunchArgument(
        'drix', default_value=TextSubstitution(text='false')
    )
    no_sim_arg = DeclareLaunchArgument(
        'no_sim', default_value=TextSubstitution(text='false')
    )
    tide_speed_factor_arg = DeclareLaunchArgument(
        'tide_speed_factor', default_value=TextSubstitution(text='10'),
        description='Tide speed multiplier (10 = ~75 min cycle, '
        '3600 = ~12 sec cycle, 1 = real-time)'
    )

    set_use_sim_time = SetParameter(
        name='use_sim_time', value=use_sim_time
    )
    # 'use_sim_time' will be set on all nodes following the line above

    launch_ben_core_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ben_project11'),
                'launch',
                'ben_core_launch.py'
            ])
        ),
        condition=is_ben,
        launch_arguments={
            'namespace': namespace,
            'enable_bridge': enable_bridge,
            'is_simulator': 'true',
        }.items()
    )

    # BizzyBoat (EchoBoat 240) sim-aware core bringup. Reuses the boat's real
    # config (nav2_overlay.yaml + base) and layers bizzyboat_sim.yaml under
    # is_simulator — no config duplication. Mirrors the ben_core include above.
    launch_bizzy_core_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bizzyboat_project11'),
                'launch',
                'bizzyboat_sim_core_launch.py'
            ])
        ),
        condition=is_bizzy,
        launch_arguments={
            'namespace': namespace,
            'is_simulator': 'true',
        }.items()
    )

    # launch_drix_core_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('drix_marine_autonomy'),
    #             'launch/drix_core_launch.py'
    #         )
    #     ),
    #     condition=IfCondition(drix),
    #     launch_arguments={
    #         'drix_number': 2,
    #         'namespace': namespace,
    #         'enable_bridge': enable_bridge
    #     }.items()
    # )

    # <rosparam if="$(arg enableBridge)"
    #     param="udp_bridge/remotes/operator/connections/default/topics/clock"
    #     ns="$(arg namespace)">{source: /clock}</rosparam>

    asv_helm_group = GroupAction(
        actions=[
            PushROSNamespace(namespace),
            SetRemap(src='helm', dst='marine/control/helm'),
            SetRemap(src='cmd_vel', dst='marine/control/cmd_vel'),
            SetRemap(src='throttle', dst='control/throttle'),
            SetRemap(src='rudder', dst='control/rudder'),
            SetRemap(
                src='have_commands',
                dst=PathJoinSubstitution(
                    ['/asv_sim', sim_name, 'have_commands']
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('asv_helm'),
                        'launch',
                        'asv_helm_launch.py'
                    ])
                ),
            )
        ],
    )

    def make_asv_sim_node(context, *args, **kwargs):
        # Resolve the namespace to a plain string. asv_sim's `platforms` param
        # is a STRING_ARRAY, but a single-element list holding a
        # LaunchConfiguration substitution normalizes to a scalar string
        # (launch_ros collapses substitution-lists), which fails the node's
        # type check and crashes it on startup. Resolving here keeps a real
        # one-element list and fully resolves the remappings.
        ns = LaunchConfiguration('namespace').perform(context)
        return [Node(
            package='asv_sim',
            executable='asv_sim',
            name='asv_sim',
            emulate_tty=True,
            parameters=[
                {'platforms': [ns]},
                {'environment.tide.speed_factor': PythonExpression(
                    expression=['float(', tide_speed_factor, ')']
                )},
            ],
            remappings=[
                (ns + '/position', ns + '/sensors/nav/position'),
                (ns + '/orientation', ns + '/sensors/nav/orientation'),
                (ns + '/velocity', ns + '/sensors/nav/velocity'),
                (ns + '/throttle', ns + '/control/throttle'),
                (ns + '/rudder', ns + '/control/rudder'),
            ],
        )]

    sim_group = GroupAction(
        actions=[
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'cw4.yaml'
                ]),
                condition=is_ben
            ),
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'ben.yaml'
                ]),
                condition=is_ben
            ),
            # BizzyBoat (EchoBoat 240): the echoboat240 hydro model + the
            # platform spawn (Massabesic pose, no current).
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'echoboat240.yaml'
                ]),
                condition=is_bizzy
            ),
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'bizzyboat.yaml'
                ]),
                condition=is_bizzy
            ),
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'drix.yaml'
                ]),
                condition=IfCondition(drix)
            ),
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'drix_2.yaml'
                ]),
                condition=IfCondition(drix)
            ),
            OpaqueFunction(function=make_asv_sim_node),
            GroupAction(
                actions=[
                    SetParameter(
                        name='sonar_frame_id',
                        value=PythonExpression(
                            expression=['"', namespace, '/mbes"']
                        )
                    ),
                    SetParameter(
                        name='ping_interval',
                        value=0.2
                    ),
                    # Ground-truth bathymetry the simulated MBES samples. For
                    # BizzyBoat-Massabesic this is the Massabesic grid; the boat's
                    # costmap starts from only the contour prior (set elsewhere).
                    SetParameter(
                        name='grid_file',
                        value=mbes_grid_file
                    ),
                    SetRemap(
                        src='detections',
                        dst=PythonExpression(
                            expression=[
                                '"/', namespace,
                                '/sensors/mbes/detections"'
                            ]
                        )
                    ),
                    SetRemap(
                        src='odom',
                        dst=PythonExpression(
                            expression=['"/', namespace, '/odom"']
                        )
                    ),
                    SetRemap(
                        src='soundings',
                        dst=PythonExpression(
                            expression=[
                                '"/', namespace,
                                '/sensors/mbes/original_soundings"'
                            ]
                        )
                    ),
                    SetRemap(
                        src='tide_level',
                        dst='/asv_sim/environment/tide_level'
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare('mbes_sim'),
                                'launch',
                                'mbes_sim_launch.py'
                            ])
                        ),
                    )
                ],
            ),
            GroupAction(
                actions=[
                    PushROSNamespace([
                        namespace,
                        '/sensors/mbes'
                    ]),
                    GroupAction(
                        actions=[
                            # detections_to_pointcloud sources the error model's
                            # pose from TF (roll/pitch from level_frame, heave
                            # from tide_frame) + /odom for SOG, since
                            # cube_bathymetry #31/#33 retired the old
                            # NavigationSensors topic path. Its frame params
                            # default to unprefixed names, so the namespaced sim
                            # MUST set them or the attitude TF lookup misses ->
                            # NaN uncertainty -> empty grid. mru_transform_node
                            # broadcasts <ns>/base_link_north_up and (via
                            # sea_surface_estimator) <ns>/map_tide.
                            SetParameter(
                                name='base_link_frame',
                                value=PythonExpression(
                                    expression=['"', namespace, '/base_link"']
                                )
                            ),
                            SetParameter(
                                name='level_frame',
                                value=PythonExpression(
                                    expression=[
                                        '"', namespace, '/base_link_north_up"'
                                    ]
                                )
                            ),
                            SetParameter(
                                name='tide_frame',
                                value=PythonExpression(
                                    expression=['"', namespace, '/map_tide"']
                                )
                            ),
                            # Speed over ground for the error model. The mbes_sim
                            # group's odom remap does not reach this group, and
                            # under this namespace a bare `odom` would resolve to
                            # <ns>/sensors/mbes/odom; point it at the real topic.
                            SetRemap(
                                src='odom',
                                dst=PythonExpression(
                                    expression=['"/', namespace, '/odom"']
                                )
                            ),
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    PathJoinSubstitution([
                                        FindPackageShare(
                                            'cube_bathymetry'
                                        ),
                                        'launch',
                                        'detections_to_pointcloud_launch.py'
                                    ])
                                )
                            ),
                        ]
                    ),
                    GroupAction(
                        actions=[
                            SetParameter(
                                name='map_frame',
                                value=PythonExpression(
                                    expression=[
                                        '"', namespace, '/map"'
                                    ]
                                )
                            ),
                            SetParameter(
                                name='cell_size',
                                value=1.0
                            ),
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(
                                    PathJoinSubstitution([
                                        FindPackageShare(
                                            'cube_bathymetry'
                                        ),
                                        'launch',
                                        'cube_bathymetry_launch.py'
                                    ])
                                )
                            ),
                        ]
                    )
                ]
            ),
        ],
        condition=UnlessCondition(no_sim),
    )

    # <node if="$(arg sim_traffic)" pkg="traffic_sim"
    #     type="traffic_sim_node.py" name="traffic_sim"
    #     ns="$(arg namespace)"/>

    # <rosparam unless="$(arg drix)" command="load"
    #     file="$(find ben_marine_autonomy)/config/ben_sim.yaml"
    #     ns="$(arg namespace)"/>

    # <rosparam if="$(arg drix)" command="load"
    #     file="$(find drix_marine_autonomy)/config/drix_sim.yaml"
    #     ns="$(arg namespace)"/>

    # <param if="$(arg enableBridge)"
    #     name="/$(arg namespace)/udp_bridge/remotes/operator/
    #     connections/default/host" value="$(arg operator_host)"/>
    # <param if="$(arg enableBridge)"
    #     name="/$(arg namespace)/udp_bridge/remotes/operator/
    #     connections/default/port" value="$(arg operator_port)"/>

    return LaunchDescription([
        namespace_arg,
        sim_name_arg,
        enable_bridge_arg,
        use_sim_time_arg,
        drix_arg,
        platform_arg,
        no_sim_arg,
        tide_speed_factor_arg,
        mbes_grid_file_arg,
        set_use_sim_time,
        launch_ben_core_include,
        launch_bizzy_core_include,
        # launch_drix_core_include,
        asv_helm_group,
        sim_group,
    ])
