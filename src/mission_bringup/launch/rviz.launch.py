"""rviz.launch.py —— 笔记本端可视化入口（只起 RViz，不起任何机器人节点）。

双机拓扑：树莓派跑 bringup/slam/mission（无头），笔记本跑本文件看 RViz。
前置：两机同一 ROS_DOMAIN_ID、同一网段，ROS2 DDS 自动发现。

用法（笔记本）:
  ros2 launch mission_bringup rviz.launch.py
可选参数:
  config:=...   # 自定义 rviz 配置，默认 config/rviz/nav.rviz
  use_sim_time:=false

单机调试也可以直接 ros2 launch mission_bringup slam.launch.py use_rviz:=true。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('mission_bringup')
    default_cfg = os.path.join(bringup_dir, 'config', 'rviz', 'nav.rviz')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('config', default_value=default_cfg))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config],
        parameters=[{'use_sim_time': use_sim_time}],
    ))

    return ld
