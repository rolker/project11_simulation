from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.actions import SetParameter

def generate_launch_description():
  robot_namespace = LaunchConfiguration('robot_namespace')
  operator_namespace = LaunchConfiguration('operator_namespace')
  enable_bridge = LaunchConfiguration('enable_bridge')
  background_chart = LaunchConfiguration('background_chart')
  use_sim_time = LaunchConfiguration('use_sim_time')
  rviz = LaunchConfiguration('rviz')
  rviz_configuration = LaunchConfiguration('rviz_configuration')

  robot_namespace_arg = DeclareLaunchArgument(
    "robot_namespace", default_value=TextSubstitution(text="ben")
  )
  operator_namespace_arg = DeclareLaunchArgument(
    "operator_namespace", default_value=TextSubstitution(text="operator")
  )
  background_chart_arg = DeclareLaunchArgument(
    "background_chart",
     default_value=PathJoinSubstitution([
      get_package_share_directory('camp'),
      'workspace', '13283', '13283_2.KAP'
    ])
  )
  enable_bridge_arg = DeclareLaunchArgument(
    "enable_bridge", default_value=TextSubstitution(text="false")
  )
  use_sim_time_arg = DeclareLaunchArgument(
    "use_sim_time", default_value=TextSubstitution(text="false")
  )

  rviz_arg = DeclareLaunchArgument(
    "rviz", default_value=TextSubstitution(text="false")
  )
  rviz_configuration_arg = DeclareLaunchArgument(
    "rviz_configuration", default_value=""
  )



  set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
  # 'use_sim_time' will be set on all nodes following the line above


  launch_operator_core_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        get_package_share_directory('project11'),
        'launch',
        'operator_core_launch.py'
      ])
    ),
    launch_arguments={
      'operator_namespace': operator_namespace,
      'robot_namespace': robot_namespace,
      'enable_bridge': enable_bridge,
      'operator_joystick': 'true'
    }.items()
  )

  launch_operator_ui_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        get_package_share_directory('project11'),
        'launch',
        'operator_ui_launch.py'
      ])
    ),
    launch_arguments={
      'namespace': operator_namespace,
      'background_chart': background_chart,
      'rviz': rviz,
      'rviz_configuration': rviz_configuration
    }.items()
  )

  def print_ros_domain_id(context):
    from launch.actions import LogInfo
    domain_id = 0
    if 'ROS_DOMAIN_ID' in context.environment:
      domain_id = context.environment['ROS_DOMAIN_ID']
    return [LogInfo(msg=f'ROS domain id: {domain_id}'),]

  return LaunchDescription([
    robot_namespace_arg,
    operator_namespace_arg,
    background_chart_arg,
    enable_bridge_arg,
    use_sim_time_arg,
    rviz_arg,
    rviz_configuration_arg,
    set_use_sim_time,
    launch_operator_core_include,
    launch_operator_ui_include,
    OpaqueFunction(
      function=print_ros_domain_id
    )
  ])


  # <group if="$(arg enableBridge)" ns="$(operator_namespace)/udp_bridge">
  #   <param name="port" value="4201"/>
  #   <param name="remotes/robot/connections/default/host" value="$(arg robot_host)"/>
  # </group>

