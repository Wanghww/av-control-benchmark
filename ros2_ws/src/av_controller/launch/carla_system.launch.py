"""
carla_system.launch.py — 第七阶段 CARLA 全集成 launch 文件

使用方式：
  # 默认参数（8 m/s，spawn_index=0）
  ros2 launch av_controller carla_system.launch.py

  # 指定速度和 spawn 点
  ros2 launch av_controller carla_system.launch.py speed:=12.0 spawn_index:=1

  # 使用 LQR 控制器
  ros2 launch av_controller carla_system.launch.py controller:=lqr

前提：CARLA 0.9.15 已在 Windows 启动（CarlaUE4.exe）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── 可配参数 ──────────────────────────────────────────────
        DeclareLaunchArgument('speed',       default_value='8.0',
                              description='目标速度 m/s'),
        DeclareLaunchArgument('spawn_index', default_value='0',
                              description='CARLA spawn_points 索引'),
        DeclareLaunchArgument('town',        default_value='Town03',
                              description='CARLA 地图名称'),

        # ── 控制器节点（和第三阶段相同，一行不改）────────────────
        Node(
            package='av_controller',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[{
                'pid_kp':    1.0,
                'pid_ki':    0.05,
                'pid_kd':    0.02,
                'pp_k':      0.5,
                'pp_ld_min': 10.0,   # 10m 前视，减少 CARLA 物理漂移引起的过激纠偏
            }]
        ),

        # ── CARLA 桥接节点（替换 sim_bridge_node）────────────────
        # 不启动 trajectory_publisher，轨迹由 carla_bridge_node 生成
        Node(
            package='av_controller',
            executable='carla_bridge_node.py',
            name='carla_bridge_node',
            output='screen',
            parameters=[{
                'carla_host':   '172.22.0.1',
                'carla_port':   2000,
                'target_speed': LaunchConfiguration('speed'),
                'spawn_index':  LaunchConfiguration('spawn_index'),
                'town':         LaunchConfiguration('town'),
            }]
        ),
    ])
