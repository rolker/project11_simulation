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
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    sim_name = LaunchConfiguration('sim_name')
    enable_bridge = LaunchConfiguration('enable_bridge')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drix = LaunchConfiguration('drix')
    no_sim = LaunchConfiguration('no_sim')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=TextSubstitution(text='ben')
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
        condition=UnlessCondition(drix),
        launch_arguments={
            'namespace': namespace,
            'enable_bridge': enable_bridge,
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

    sim_group = GroupAction(
        actions=[
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'cw4.yaml'
                ]),
                condition=UnlessCondition(drix)
            ),
            SetParametersFromFile(
                PathJoinSubstitution([
                    FindPackageShare('asv_sim'), 'config', 'ben.yaml'
                ]),
                condition=UnlessCondition(drix)
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
            Node(
                package='asv_sim',
                executable='asv_sim',
                name='asv_sim',
                emulate_tty=True,
                parameters=[{'platforms': ['ben']}],
                remappings=[
                    (
                        PathJoinSubstitution([namespace, 'position']),
                        PathJoinSubstitution([
                            namespace, 'sensors', 'nav', 'position'
                        ])
                    ),
                    (
                        PathJoinSubstitution([namespace, 'orientation']),
                        PathJoinSubstitution([
                            namespace, 'sensors', 'nav', 'orientation'
                        ])
                    ),
                    (
                        PathJoinSubstitution([namespace, 'velocity']),
                        PathJoinSubstitution([
                            namespace, 'sensors', 'nav', 'velocity'
                        ])
                    ),
                    (
                        PathJoinSubstitution([namespace, 'throttle']),
                        PathJoinSubstitution([
                            namespace, 'control', 'throttle'
                        ])
                    ),
                    (
                        PathJoinSubstitution([namespace, 'rudder']),
                        PathJoinSubstitution([
                            namespace, 'control', 'rudder'
                        ])
                    )
                ]
            ),
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
                            SetParameter(
                                name='sensors.default.topics.position',
                                value=PythonExpression(
                                    expression=[
                                        '"/', namespace,
                                        '/sensors/nav/position"'
                                    ]
                                )
                            ),
                            SetParameter(
                                name='sensors.default.topics.orientation',
                                value=PythonExpression(
                                    expression=[
                                        '"/', namespace,
                                        '/sensors/nav/orientation"'
                                    ]
                                )
                            ),
                            SetParameter(
                                name='sensors.default.topics.velocity',
                                value=PythonExpression(
                                    expression=[
                                        '"/', namespace,
                                        '/sensors/nav/velocity"'
                                    ]
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
            # sea_surface_estimator: estimates tide from odom z for
            # the map_tide frame used by depth/costmap adjustments
            GroupAction(
                actions=[
                    PushROSNamespace(namespace),
                    LifecycleNode(
                        package='mru_transform',
                        executable='sea_surface_estimator',
                        name='sea_surface_estimator',
                        namespace='',
                        respawn=True,
                        respawn_delay=2,
                        emulate_tty=True,
                    ),
                    LifecycleTransition(
                        lifecycle_node_names=(
                            PythonExpression(
                                expression=[
                                    '"/',
                                    namespace,
                                    '/sea_surface_estimator"',
                                ],
                            ),
                        ),
                        transition_ids=(
                            Transition.TRANSITION_CONFIGURE,
                            Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
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
        no_sim_arg,
        set_use_sim_time,
        launch_ben_core_include,
        # launch_drix_core_include,
        asv_helm_group,
        sim_group,
    ])
