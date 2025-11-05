
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  background_chart = LaunchConfiguration('background_chart')
  enable_bridge = LaunchConfiguration('enable_bridge')

  background_chart_arg = DeclareLaunchArgument(
    "background_chart", default_value=PathJoinSubstitution(
      [FindPackageShare('camp'), 'workspace', '13283', '13283_2.KAP']
    )
  )

  enable_bridge_arg = DeclareLaunchArgument(
    "enable_bridge", default_value=TextSubstitution(text="false")
  )


  launch_sim_robot_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare('project11_simulation'),
        'launch',
        'sim_robot_launch.py'
      ])
    ),
    launch_arguments={
      'namespace': 'ben',
      'enable_bridge': 'false',
    }.items()
  )


  launch_sim_operator_group = GroupAction(
    actions=[
      SetEnvironmentVariable(
        name = 'ROS_DOMAIN_ID',
        value =  '1',
        condition = IfCondition(enable_bridge)
      ),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([
            FindPackageShare('project11_simulation'),
            'launch',
            'sim_operator_launch.py'
          ])
        ),
        launch_arguments={
          'robot_namespace': 'ben',
          'operator_namespace': 'operator',
          'enable_bridge': 'false',
          'background_chart': background_chart,
          'rviz': 'true',
          'rviz_configuration': PathJoinSubstitution([
            FindPackageShare('ben_project11'),
            'config',
            'ben.rviz'
          ])
        }.items()
      ),

      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([
            FindPackageShare('project11_simulation'),
            'launch',
            'udp_bridge_launch.py'
          ])
        ),
        condition=IfCondition(enable_bridge)
      )
    ]
  )

  return LaunchDescription([
    background_chart_arg,
    enable_bridge_arg,
    LogInfo(
      condition=IfCondition(enable_bridge),
      msg=TextSubstitution(text="Bridge enabled")
    ),
    launch_sim_robot_include,
    launch_sim_operator_group
  ])


