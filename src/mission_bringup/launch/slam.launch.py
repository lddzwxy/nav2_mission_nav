"""slam.launch.py —— 建图：bringup + SLAM Toolbox(online async)。

机器人端（树莓派）无头运行；笔记本端另起 rviz.launch.py 可视化。
用法:
  ros2 launch mission_bringup slam.launch.py
可选参数:
  use_rviz:=false   # 单机调试想在机器人上开 RViz 时置 true
遥控建图（另开终端）:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard   # 或手柄
保存地图:
  ros2 run nav2_map_server map_saver_cli -f maps/lab_5x5
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('mission_bringup')
    slam_params = os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    use_rviz = LaunchConfiguration('use_rviz')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))

    # 底盘 + 雷达 + TF
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup.launch.py')),
        launch_arguments={'use_rviz': use_rviz}.items(),
    ))

    # SLAM Toolbox async 建图
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params,
        }.items(),
    ))

    return ld
