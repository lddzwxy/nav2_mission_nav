# 踩坑记录（troubleshooting.md）

> 每个坑按"现象 → 排查 → 根因 → 处理"写。

## Jazzy 参数坑（从 Humble 教程照抄会踩）

### 目标容差改了没反应
- 现象：改 DWB 里 `xy_goal_tolerance` 无效，到点精度没变化。
- 根因：Jazzy 起目标容差在 `controller_server.general_goal_checker.*`，DWB 内同名参数不生效。
- 处理：改 `general_goal_checker` 的 `xy_goal_tolerance` / `yaw_goal_tolerance`。

### AMCL 不认 odom_model_type
- 现象：配置 `odom_model_type: diff` 被忽略/告警。
- 根因：Jazzy 用 `robot_model_type: "nav2_amcl::DifferentialMotionModel"`。
- 处理：换成 `robot_model_type`，`odom_alpha1~5` 也要写成 `alpha1~5`。

### 自定义 BT 不加载
- 现象：`default_bt_xml_filename` 写了自定义 XML，行为树还是默认。
- 根因：该参数已废弃，Jazzy 用 `default_nav_to_pose_bt_xml`。
- 处理：换参数名；路径用 `$(find-pkg-share mission_bringup)/config/bt/...` 注入。

### 代价地图参数名不识别
- 现象：`obstacle_range` / `raytrace_range` 不生效。
- 根因：Jazzy 为 `obstacle_max_range` / `raytrace_max_range`。
- 处理：换名，且 `raytrace_max_range ≥ obstacle_max_range`。

## 系统类

### 导航 bringup 中止：parameter 'observation_sources' is not initialized
- 现象：`mission.launch.py` 起到最后，`collision_monitor` configure 报 `parameter 'observation_sources' is not initialized`，随后 `lifecycle_manager_navigation: Failed to bring up all requested nodes. Aborting bringup.` 整个 Nav2 不激活。
- 根因：Jazzy 的 `nav2_bringup/navigation_launch.py` **无条件**启动 `route_server`、`collision_monitor`、`docking_server`（没有开关参数），我们的 `nav2_params.yaml` 缺这三段 → 节点 configure 时读不到必填参数。
- 处理：`nav2_params.yaml` 补 `collision_monitor`（`observation_sources` + `polygons`）和 `docking_server`（`dock_plugins`）两段，照官方 `nav2_bringup/params/nav2_params.yaml` 抄，帧名改成 `base_link`。`route_server` 没图也能 configure 过，可不管。
- 顺带收益：`collision_monitor` 配好就是方案 §6.6 的"激光安全区急停"加分项（FootprintApproach 模式）。

### EKF 刷屏：Failed to meet update rate
- 现象：`ekf_filter_node` 不停报 `Failed to meet update rate! Took 0.02~0.12s`。
- 根因：`ekf.yaml` 的 `frequency: 50.0` 要求 20ms 一个周期，树莓派上 Nav2 全家桶 + SLAM 抢 CPU，单次 update 就要 21~120ms。
- 处理：`frequency: 30.0`（robot_localization 上游默认值，室内低速导航足够）。仍超时再降 25 或把 IMU 降频。

### 激光点云整体转了 180°（障碍出现在车尾方向）
- 现象：RViz 里激光与地图/实际环境差半个圈，推车前进时前方障碍显示在车后。
- 根因：雷达 0° 基准朝后安装（如 USB 线朝车头），`base_link → laser_frame` 的 yaw 差 π。
- 处理：改 `bringup.launch.py` 里静态 TF 的 `--yaw` 为 `3.14159`（反了就改回 0 或 -3.14159）。**不要**用驱动的 `inverted` 参数——那是给倒装（上下颠倒、扫描镜像）用的，纯水平转 180° 用它会变成镜像错误。
- 验证：推车直行，RViz 激光贴墙不斜、前方障碍在车头；`ros2 run tf2_ros tf2_echo base_link laser_frame` 看 yaw≈π。

### 两个节点抢同一个串口（launch 参数撞名）
- 现象：雷达报 `cannot bind to the specified serial port '/dev/ttyUSB1'`，底盘也报 `Failed to open /dev/ttyUSB1`——明明配置写的不是同一个口。
- 根因：ROS 2 launch 配置名是**全局作用域**。`mission_bringup/bringup.launch.py` 若声明 `serial_port`（给雷达），include 进来的 `diff_drive_bringup/base.launch.py` 里同名参数的默认值会被外层覆盖，底盘也去开雷达的口。
- 处理：外层雷达口改名 `lidar_port`，include 底盘时**显式传** `launch_arguments={'serial_port': base_port}`。udev 固定名：雷达 `/dev/rplidar`、底盘 `/dev/stm32`。

### base_driver 崩溃：can't subtract times with different time sources [1 != 2]
- 现象：底盘串口打开失败后约 1s，`base_driver_node` 抛 `std::runtime_error` 进程挂掉（exit code -6）。
- 根因：`rclcpp::Time` 成员默认构造是 `RCL_SYSTEM_TIME`(2)，节点 `now()` 是 `RCL_ROS_TIME`(1)。`last_imu_time_` 若不在构造函数里初始化，串口没帧进来时 diag 定时器算 `(now() - last_imu_time_)` 就炸。
- 处理：构造函数里 `last_cmd_time_` / `last_status_time_` / `last_imu_time_` **三个都要** `= now()`。通用教训：类里的 `rclcpp::Time` 成员一律在构造时用 `now()` 初始化，绝不能裸声明。

### 雷达 launch 报 executable 'rplidar_node' not found
- 现象：`ros2 launch mission_bringup slam.launch.py` 报 `executable 'rplidar_node' not found on the libexec directory '/opt/ros/jazzy/lib/rplidar_ros'`。
- 根因：Jazzy 的 `ros-jazzy-rplidar-ros` 发行自社区 fork `allenh1/rplidar_ros`（v2.1.0），可执行名是 **`rplidar_composition`**；`rplidar_node` 是 Slamtec 上游（Humble 发行版）的名字。
- 处理：launch 里 `executable='rplidar_composition'`；参数按 allenh1 版：`serial_baudrate` 为 int、无 `channel_type`。可用 `ls /opt/ros/jazzy/lib/rplidar_ros/` 确认实际可执行名。

### mission_manager 链接失败：undefined reference to `YAML::...`
- 现象：编译能过，链接时 `waypoint_loader.cpp.o` 报一长串 `undefined reference to YAML::LoadFile / vtable for YAML::...`。
- 根因：`yaml_cpp_vendor` 是 vendor 包，`ament_target_dependencies(... yaml_cpp_vendor)` 只提供头文件路径，**不链接 libyaml-cpp**。
- 处理：`find_package(yaml-cpp REQUIRED)` + `target_link_libraries(mission_manager_node yaml-cpp)`（新版本目标名是 `yaml-cpp::yaml-cpp`，两个都试）。

### 车不动，节点都在
- 排查：`ros2 lifecycle list /controller_server` 看是否 active。
- 根因：Nav2 是 managed node，launch 漏了 lifecycle_manager。
- 处理：用 `navigation_launch.py` 统一激活；启动后逐个确认 active。

### 一直转圈不走
- 排查：RViz 看 local costmap 是否整片致命区。
- 根因：inflation_radius 过大 / 传感器把自身机臂扫成障碍。
- 处理：降 inflation；给 `/scan` 加自遮挡滤除或调 `obstacle_max_range`。

### 路径有了但不跟
- 排查：`ros2 topic hz /cmd_vel`；`ros2 node list | grep teleop`。
- 根因：teleop 抢发 `/cmd_vel` 或 controller 频率过低。
- 处理：关 teleop；`controller_frequency: 20.0`。

### map→odom 断链
- 排查：`ros2 run tf2_tools view_frames`；`tf2_echo map odom`。
- 根因：AMCL 没收到 `/scan`，或与 SLAM Toolbox 同时发 map→odom。
- 处理：建图/导航二选一起；查 `/scan` hz 与 frame_id=laser_frame。
