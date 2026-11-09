#pragma once

#include <string>
#include <vector>

namespace mission_manager
{

enum class WaypointAction
{
  kNavigate,
  kWait,
  kSpin,
  kCustom,
};

struct Waypoint
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  WaypointAction action{WaypointAction::kNavigate};
  double xy_tolerance{0.10};     // 仅 navigate
  double yaw_tolerance{0.08};    // 仅 navigate
  double duration_sec{0.0};      // 仅 wait
  double target_yaw{0.0};        // 仅 spin，相对旋转量（rad）
};

struct MissionConfig
{
  std::string name{"mission"};
  bool loop{false};
  int max_retries{2};
  bool skip_on_fail{true};
  double goal_timeout_sec{60.0};
  std::vector<Waypoint> waypoints;
};

// 从 YAML 文件加载任务；失败返回 false 并填 error
bool LoadMissionFromYaml(const std::string & path, MissionConfig & out, std::string & error);

const char * ToString(WaypointAction action);

}  // namespace mission_manager
