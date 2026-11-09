# MissionStatus.msg —— 任务状态机对外快照
std_msgs/Header header

# 状态机枚举（与 mission_manager 中 State 一一对应）
uint8 STATE_LOADING=0
uint8 STATE_IDLE=1
uint8 STATE_NAVIGATING=2
uint8 STATE_ARRIVED=3
uint8 STATE_DO_ACTION=4
uint8 STATE_RETRYING=5
uint8 STATE_SKIPPED=6
uint8 STATE_PAUSED=7
uint8 STATE_DONE=8
uint8 STATE_FAILED=9
uint8 STATE_ABORTED=10

string mission_name
uint8 state
uint8 current_index
uint8 total_waypoints
string current_wp_id
uint8 retry_count
float32 progress         # 0~1，(已完成路点数)/total
string last_error
builtin_interfaces/Duration eta_hint   # 预计剩余时间（无估计则为 0）
