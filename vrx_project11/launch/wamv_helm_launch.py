from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import LifecycleTransition

from lifecycle_msgs.msg import Transition

def generate_launch_description():

    return LaunchDescription([
        LifecycleNode(
            package='vrx_marine_autonomy',
            executable='wamv_helm',
            name='wamv_helm',
            namespace='',
            respawn=True,
            respawn_delay=2,
            emulate_tty=True
        ),
        LifecycleTransition(
            lifecycle_node_names=(
                PythonExpression(
                    expression = [
                        '"',
                        LaunchConfiguration("ros_namespace", default=''),
                        '" + "/wamv_helm"'
                    ],
                ),
            ),
            transition_ids=(
                Transition.TRANSITION_CONFIGURE,
                Transition.TRANSITION_ACTIVATE,
            )
        ),
    ])
