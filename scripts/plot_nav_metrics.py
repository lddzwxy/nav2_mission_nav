#!/usr/bin/env python3
"""plot_nav_metrics.py —— 从 rosbag 提取导航指标并出图（方案 §9.1/§6.7 复盘用）。

用法:
  python3 plot_nav_metrics.py --bag patrol_run --out metrics.png

提取: /mission_status（耗时/重试）、/plan（路径长度）、/local_plan（重规划次数）。
依赖: rosbag2_py, matplotlib（树莓派/笔记本均可）。
"""

import argparse
import math
import sys
from collections import defaultdict


def path_length(poses):
    total = 0.0
    for a, b in zip(poses, poses[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total


def main():
    ap = argparse.ArgumentParser(description='导航指标出图')
    ap.add_argument('--bag', required=True, help='rosbag2 目录')
    ap.add_argument('--out', default='metrics.png', help='输出图片路径')
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from nav_msgs.msg import Path
        from mission_msgs.msg import MissionStatus
    except ImportError as e:
        print(f'依赖缺失: {e}\n  pip/pipx 装 matplotlib，ros-jazzy-rosbag2 提供 rosbag2_py', file=sys.stderr)
        sys.exit(1)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    plan_lens, replans = [], 0
    progress_series = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/plan' and topic_types.get(topic) == 'nav_msgs/msg/Path':
            msg = deserialize_message(data, Path)
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            plan_lens.append(path_length(pts))
        elif topic == '/local_plan' and topic_types.get(topic) == 'nav_msgs/msg/Path':
            replans += 1
        elif topic == '/mission_status' and topic_types.get(topic) == 'mission_msgs/msg/MissionStatus':
            msg = deserialize_message(data, MissionStatus)
            progress_series.append((msg.header.stamp.sec, msg.progress))

    if not plan_lens and not progress_series:
        print('bag 中没有 /plan /local_plan /mission_status，检查录制话题列表', file=sys.stderr)
        sys.exit(1)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    if plan_lens:
        axes[0].plot(plan_lens, marker='o')
        axes[0].set_title(f'全局路径长度（{len(plan_lens)} 次规划）')
        axes[0].set_ylabel('m')
    if progress_series:
        t0 = progress_series[0][0]
        axes[1].plot([t - t0 for t, _ in progress_series],
                     [p for _, p in progress_series])
        axes[1].set_title('任务进度（重规划次数 %d）' % replans)
        axes[1].set_ylabel('progress 0~1')
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f'已保存 {args.out}；总重规划次数 {replans}')


if __name__ == '__main__':
    main()
