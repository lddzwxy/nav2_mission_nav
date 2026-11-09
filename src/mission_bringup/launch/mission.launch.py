"""mission.launch.py —— 一键演示：导航栈 + Mission Manager（可选 Web 桥）。

用法:
  ros2 launch mission_bringup mission.launch.py \
      map:=/path/to/lab_5x5.yaml \
      mission:=/path/to/five_points.yaml
参数:
  map        地图 yaml 路径（必填）
  mission    路点任务 yaml，默认 config/waypoints/five_points.yaml
  use_rviz   是否起 RViz，默认 false（机器人端无头；笔记本端跑 rviz.launch.py）
  use_web    是否起简易 Web 面板，默认 false（可选加分项）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('mission_bringup')

    map_file = LaunchConfiguration('map')
    mission_file = LaunchConfiguration('mission')
    use_rviz = LaunchConfiguration('use_rviz')
    use_web = LaunchConfiguration('use_web')

    default_mission = os.path.join(
        bringup_dir, 'config', 'waypoints', 'five_points.yaml')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('map', description='地图 yaml 绝对路径'))
    ld.add_action(DeclareLaunchArgument('mission', default_value=default_mission))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_web', default_value='false'))

    # 导航全栈（含 bringup + AMCL + Nav2）
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav.launch.py')),
        launch_arguments={'map': map_file, 'use_rviz': use_rviz}.items(),
    ))

    # 自研任务调度器
    ld.add_action(Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[{'mission_file': mission_file, 'tick_rate': 5.0}],
    ))

    # 可选 Web 桥（时间紧不做，真机成功率优先）
    # ld.add_action(Node(
    #     package='mission_web', executable='web_bridge', name='web_bridge',
    #     condition=IfCondition(use_web), output='screen',
    # ))

    return ld
