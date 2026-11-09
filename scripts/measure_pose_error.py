#!/usr/bin/env python3
"""measure_pose_error.py —— 到点误差统计（方案 §9.3）。

两种用法：
1) 手工录入：先建 CSV（scripts/pose_samples.csv，列 wp_id, dx, dy, dyaw_deg），然后
       python3 measure_pose_error.py --csv pose_samples.csv
2) 在线采样：任务跑完后立即执行，从 /mission_status 附近采当前位姿与目标对比
   （需要 RViz/卷尺手工核对，本脚本只做汇总）

输出：每路点均值/P95、总位置误差、航向误差，可直接贴 README。
"""

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict


def pct(values, p):
    """简单百分位（近似，样本少时足够）"""
    if not values:
        return float('nan')
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] + (s[c] - s[f]) * (k - f)


def main():
    ap = argparse.ArgumentParser(description='到点误差统计')
    ap.add_argument('--csv', required=True, help='pose_samples.csv 路径')
    args = ap.parse_args()

    per_wp = defaultdict(list)   # wp_id -> [(pos_err, yaw_err)]
    all_pos, all_yaw = [], []

    with open(args.csv, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            wp = row['wp_id'].strip()
            dx = float(row['dx'])
            dy = float(row['dy'])
            dyaw = float(row['dyaw_deg'])
            pos_err = math.hypot(dx, dy)
            per_wp[wp].append((pos_err, abs(dyaw)))
            all_pos.append(pos_err)
            all_yaw.append(abs(dyaw))

    if not all_pos:
        print('CSV 无有效样本', file=sys.stderr)
        sys.exit(1)

    print(f'样本数: {len(all_pos)}\n')
    print(f'{"路点":<10}{"次数":>4}{"位置均值(cm)":>12}{"位置P95(cm)":>12}'
          f'{"航向均值(°)":>12}{"航向P95(°)":>12}')
    for wp in sorted(per_wp):
        items = per_wp[wp]
        pos = [x[0] for x in items]
        yaw = [x[1] for x in items]
        print(f'{wp:<10}{len(items):>4}'
              f'{statistics.mean(pos) * 100:>12.1f}{pct(pos, 95) * 100:>12.1f}'
              f'{statistics.mean(yaw):>12.1f}{pct(yaw, 95):>12.1f}')

    print('-' * 62)
    print(f'{"总体":<10}{len(all_pos):>4}'
          f'{statistics.mean(all_pos) * 100:>12.1f}{pct(all_pos, 95) * 100:>12.1f}'
          f'{statistics.mean(all_yaw):>12.1f}{pct(all_yaw, 95):>12.1f}')

    pos_ok = pct(all_pos, 95) < 0.10
    yaw_ok = pct(all_yaw, 95) < 5.0
    print()
    print(f'验收（P95）：位置 {"达标 <10cm" if pos_ok else "未达标 >=10cm"}，'
          f'航向 {"达标 <5°" if yaw_ok else "未达标 >=5°"}')


if __name__ == '__main__':
    main()
