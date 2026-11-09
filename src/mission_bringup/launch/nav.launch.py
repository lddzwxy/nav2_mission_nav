"""nav.launch.py —— 导航：bringup + map_server + AMCL + Nav2 全栈。

用法:
  ros2 launch mission_bringup nav.launch.py map:=/path/to/lab_5x5.yaml
参数:
  map        地图 yaml 路径（必填）
  params     Nav2 参数文件，默认 config/nav2_params.yaml
  use_rviz   是否起 RViz，默认 false（机器人端无头；笔记本端跑 rviz.launch.py）
  autostart  lifecycle 自动激活，默认 true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('mission_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    default_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('map', description='地图 yaml 绝对路径'))
    ld.add_action(DeclareLaunchArgument('params', default_value=default_params))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))

    # 底盘 + 雷达 + TF
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
        launch_arguments={'use_rviz': 'false'}.items(),
    ))

    # map_server + AMCL（nav2_bringup localization_launch）
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'autostart': autostart,
            'use_sim_time': 'false',
        }.items(),
    ))

    # Nav2 导航栈（planner / controller / bt_navigator / behavior_server /
    # velocity_smoother / lifecycle_manager）——由 navigation_launch 统一拉起与激活
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'autostart': autostart,
            'use_sim_time': 'false',
        }.items(),
    ))

    # RViz（单机调试用；正常双机跑法是笔记本端 rviz.launch.py）
    rviz_cfg = os.path.join(bringup_dir, 'config', 'rviz', 'nav.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_cfg],
        output='screen',
    ))

    return ld
