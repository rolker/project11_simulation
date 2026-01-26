from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    tf_prefix = LaunchConfiguration('tf_prefix', default=namespace)

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value=TextSubstitution(text="wamv")
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value=namespace
        ),
        SetParameter(
            name='use_sim_time',
            value=True
        ),
        GroupAction(
            actions=[
                PushROSNamespace(namespace),
                SetParametersFromFile(
                    filename=PathJoinSubstitution([
                        FindPackageShare('vrx_marine_autonomy'),
                        'config',
                        'wamv.yaml'
                    ])
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('marine_autonomy'),
                            'launch',
                            'robot_core_launch.py'
                        ])
                    ),
                    launch_arguments={
                        'namespace': namespace,
                    }.items()
                ),
                GroupAction(
                    actions=[
                        PushROSNamespace('sensors/gps/gps'),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                    FindPackageShare('mru_transform'),
                                    'launch',
                                    'nav_sat_fix_to_velocity_launch.py'
                                ])
                            ),
                        ),
                    ]
                ),
                Node(
                    package='mru_transform',
                    executable='mru_transform_node',
                    name='mru_transform',
                    emulate_tty=True,
                    parameters=[
                        {'base_frame': [tf_prefix, '/base_link']},
                        {'map_frame': [tf_prefix, '/map']},
                        {'odom_frame': [tf_prefix, '/odom']}
                    ],
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='wamv_static_tf_broadcaster',
                    arguments=['--frame-id', [tf_prefix, '/base_link'], '--child-frame-id', [tf_prefix, '/wamv/base_link']]
                ),
                GroupAction(
                    actions=[
                        SetRemap(src='cmd_vel', dst='marine_autonomy/control/cmd_vel'),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                PathJoinSubstitution([
                                    FindPackageShare('vrx_project11'),
                                    'launch',
                                    'wamv_helm_launch.py'
                                ])
                            ),
                        ),
                    ]
                ),
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="mbes_bridge",
                )
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('vrx_marine_autonomy'),
                    'launch',
                    'nav2_bringup_launch.py'
                ])
            ),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'true',
                'use_composition': 'False',
                'use_respawn': 'True',
                'use_sim_time': 'true',
            }.items()
        )
    ])
