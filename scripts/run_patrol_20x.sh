#!/usr/bin/env bash
# run_patrol_20x.sh —— 20 轮巡航验收（方案 §9.2）
# 前置：mission.launch.py 已起、地图与路点就绪。
# 用法: ./run_patrol_20x.sh [轮数，默认20] [单轮超时秒，默认300]
# 结果追加到 docs/field_test_log.md

set -u

ROUNDS="${1:-20}"
TIMEOUT="${2:-300}"
LOG_FILE="$(dirname "$0")/../docs/field_test_log.md"
MISSION_FILE="${MISSION_FILE:-}"   # 空则用 launch 默认

ok_count=0
fail_count=0

if [ ! -f "$LOG_FILE" ]; then
  echo "| 轮次 | 结果 | 失败路点 | 错误 | 重试次数 | 耗时 | 备注 |" > "$LOG_FILE"
  echo "|------|------|----------|------|----------|------|------|" >> "$LOG_FILE"
fi

get_status_field() {
  # /mission_status 单帧快照取字段（field 名作参数）
  timeout 3 ros2 topic echo --once /mission_status 2>/dev/null | grep "$1" | head -1 | awk '{print $2}'
}

wait_mission_end() {
  local waited=0
  while [ "$waited" -lt "$TIMEOUT" ]; do
    local state
    state=$(get_status_field "state:")
    # 8=DONE 9=FAILED 10=ABORTED（见 MissionStatus.msg）
    case "$state" in
      8|9|10) echo "$state"; return 0 ;;
    esac
    sleep 2
    waited=$((waited + 2))
  done
  echo "timeout"
  return 1
}

echo "=== 巡航验收：${ROUNDS} 轮，单轮超时 ${TIMEOUT}s ==="

for i in $(seq 1 "$ROUNDS"); do
  echo "--- 第 ${i}/${ROUNDS} 轮 ---"
  start_ts=$(date +%s)

  if [ -n "$MISSION_FILE" ]; then
    ros2 service call /mission_command mission_msgs/srv/MissionCommand \
      "{command: 'START', mission_file: '${MISSION_FILE}'}" > /dev/null
  else
    ros2 service call /mission_command mission_msgs/srv/MissionCommand \
      "{command: 'START', mission_file: ''}" > /dev/null
  fi

  final_state=$(wait_mission_end)
  end_ts=$(date +%s)
  duration=$((end_ts - start_ts))

  wp_id=$(get_status_field "current_wp_id:")
  err=$(get_status_field "last_error:")
  retry=$(get_status_field "retry_count:")

  if [ "$final_state" = "8" ]; then
    ok_count=$((ok_count + 1))
    echo "| ${i} | OK | - | - | ${retry} | ${duration}s | |" >> "$LOG_FILE"
    echo "OK (${duration}s)"
  else
    fail_count=$((fail_count + 1))
    echo "| ${i} | FAIL | ${wp_id} | ${err} | ${retry} | ${duration}s | state=${final_state} |" >> "$LOG_FILE"
    echo "FAIL state=${final_state} wp=${wp_id} err=${err}"
    # 失败后尝试恢复现场：ABORT 清状态，下一轮重新 START
    ros2 service call /mission_command mission_msgs/srv/MissionCommand \
      "{command: 'ABORT', mission_file: ''}" > /dev/null 2>&1
  fi

  sleep 5
done

total=$((ok_count + fail_count))
if [ "$total" -gt 0 ]; then
  rate=$((ok_count * 100 / total))
else
  rate=0
fi

echo ""
echo "=== 结果：${ok_count}/${total} 成功，成功率 ${rate}% ==="
echo "$(date '+%F %T') 巡航验收 ${ok_count}/${total} (${rate}%)" >> "$LOG_FILE"
