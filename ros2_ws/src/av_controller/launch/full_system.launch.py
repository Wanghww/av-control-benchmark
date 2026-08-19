from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='av_controller',
            executable='trajectory_publisher',
            name='trajectory_publisher',
            output='screen'
        ),
        Node(
            package='av_controller',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{
                'pid_kp':    1.5,
                'pid_ki':    0.1,
                'pid_kd':    0.05,
                'pp_k':      0.3,
                'pp_ld_min': 3.0,
            }]
        ),
        Node(
            package='av_controller',
            executable='sim_bridge_node.py',
            name='sim_bridge_node',
            output='screen'
        ),
    ])
