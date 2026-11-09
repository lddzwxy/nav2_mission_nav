"""bringup.launch.py —— 底盘 + 雷达 + 静态 TF 一键起（不含 SLAM/Nav2）。

机器人端（树莓派）默认无头：不启动 rviz2，可视化在笔记本端跑 rviz.launch.py。
用法:
  ros2 launch mission_bringup bringup.launch.py
可选参数:
  use_base:=false     # 只起雷达+TF，底盘另开（调试用）
  use_rviz:=false     # 单机调试想在机器人上开 RViz 时置 true
  lidar_port:=/dev/rplidar   # 雷达串口（udev 固定名）
  base_port:=/dev/stm32      # 底盘 STM32 串口（udev 固定名）

注意：launch 配置名是全局作用域。这里故意把雷达口叫 lidar_port 而不是 serial_port，
否则会与 diff_drive_bringup/base.launch.py 的 serial_port 参数撞名，
把底盘的串口也覆盖成雷达口（两个节点抢同一个 /dev/ttyUSB*）。
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
    # 项目一底盘 launch（独立仓库 ros2_diff_drive，需先 build + source）
    try:
        base_dir = get_package_share_directory('diff_drive_bringup')
        base_launch = os.path.join(base_dir, 'launch', 'base.launch.py')
        ekf_launch = os.path.join(base_dir, 'launch', 'ekf.launch.py')
        has_base = os.path.exists(base_launch)
    except Exception:
        has_base = False

    use_base = LaunchConfiguration('use_base')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_port = LaunchConfiguration('lidar_port')
    base_port = LaunchConfiguration('base_port')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_base', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false'))
    ld.add_action(DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'))
    ld.add_action(DeclareLaunchArgument('base_port', default_value='/dev/stm32'))

    # ---- 静态 TF：base_link → laser_frame（安装位姿实测后改这里的数值）----
    # yaw=π：雷达 0° 基准朝后安装（USB 线朝车头）时的修正。
    # 验证：推车前进，RViz 激光贴墙不斜、前方障碍出现在车头方向。
    # 注意：这里改 TF 而不是驱动的 inverted 参数——inverted 是给倒装(镜像)用的，
    # 纯水平转 180° 用 inverted 会把扫描变成镜像，越修越歪。
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['--x', '0.08', '--y', '0.0', '--z', '0.15',
                   '--yaw', '3.14159', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
    ))

    # ---- 静态 TF：base_link → camera_link（项目三预留，默认关闭）----
    # ld.add_action(Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_to_camera_tf',
    #     arguments=['--x', '0.10', '--y', '0.0', '--z', '0.20',
    #                '--yaw', '0', '--pitch', '0', '--roll', '0',
    #                '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    # ))

    # ---- RPLIDAR A1（apt: ros-jazzy-rplidar-ros）----
    # 注意：Jazzy 的 ros-jazzy-rplidar-ros 发行自 allenh1/rplidar_ros（v2.1.0），
    # 可执行名是 rplidar_composition（不是 Slamtec 上游的 rplidar_node）。
    # 参数以 allenh1 launch/rplidar.launch.py 为准：波特率是 int，无 channel_type。
    ld.add_action(Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
    ))

    # ---- 项目一底盘 + EKF（含 /odom_raw /imu/data /odometry/filtered）----
    if has_base:
        # 显式传 serial_port，避免继承到雷达口的旧值
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            condition=IfCondition(use_base),
            launch_arguments={'serial_port': base_port}.items(),
        ))
        if os.path.exists(ekf_launch):
            ld.add_action(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_launch),
                condition=IfCondition(use_base),
            ))
    else:
        # 找不到 diff_drive_bringup 时给出明确指引而不是静默失败
        from launch.actions import LogInfo
        ld.add_action(LogInfo(
            msg='[mission_bringup] 警告: 未找到 diff_drive_bringup，'
                '请先编译并 source 项目一 ros2_diff_drive，或 use_base:=false',
            condition=IfCondition(use_base),
        ))

    # ---- RViz ----
    rviz_cfg = os.path.join(bringup_dir, 'config', 'rviz', 'nav.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg] if os.path.exists(rviz_cfg) else [],
        condition=IfCondition(use_rviz),
    ))

    return ld
