#include "mission_manager/waypoint.hpp"

#include <fstream>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace mission_manager
{

const char * ToString(WaypointAction action)
{
  switch (action) {
    case WaypointAction::kNavigate: return "navigate";
    case WaypointAction::kWait: return "wait";
    case WaypointAction::kSpin: return "spin";
    case WaypointAction::kCustom: return "custom";
  }
  return "unknown";
}

WaypointAction ParseAction(const std::string & s)
{
  if (s == "wait") {return WaypointAction::kWait;}
  if (s == "spin") {return WaypointAction::kSpin;}
  if (s == "custom") {return WaypointAction::kCustom;}
  return WaypointAction::kNavigate;
}

bool LoadMissionFromYaml(const std::string & path, MissionConfig & out, std::string & error)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    error = std::string("YAML 加载失败: ") + e.what();
    return false;
  }

  try {
    if (root["mission"]) {
      const auto m = root["mission"];
      if (m["name"]) {out.name = m["name"].as<std::string>();}
      if (m["loop"]) {out.loop = m["loop"].as<bool>();}
      if (m["max_retries"]) {out.max_retries = m["max_retries"].as<int>();}
      if (m["skip_on_fail"]) {out.skip_on_fail = m["skip_on_fail"].as<bool>();}
      if (m["goal_timeout_sec"]) {out.goal_timeout_sec = m["goal_timeout_sec"].as<double>();}
    }

    if (!root["waypoints"] || !root["waypoints"].IsSequence() || root["waypoints"].size() == 0) {
      error = "缺少 waypoints 列表或为空";
      return false;
    }

    out.waypoints.clear();
    for (const auto & w : root["waypoints"]) {
      Waypoint wp;
      wp.id = w["id"] ? w["id"].as<std::string>() : "wp" + std::to_string(out.waypoints.size());
      if (!w["pose"]) {
        error = "路点 " + wp.id + " 缺少 pose";
        return false;
      }
      wp.x = w["pose"]["x"].as<double>();
      wp.y = w["pose"]["y"].as<double>();
      wp.yaw = w["pose"]["yaw"] ? w["pose"]["yaw"].as<double>() : 0.0;
      wp.action = ParseAction(w["action"] ? w["action"].as<std::string>() : "navigate");
      if (w["xy_tolerance"]) {wp.xy_tolerance = w["xy_tolerance"].as<double>();}
      if (w["yaw_tolerance"]) {wp.yaw_tolerance = w["yaw_tolerance"].as<double>();}
      if (w["duration_sec"]) {wp.duration_sec = w["duration_sec"].as<double>();}
      if (w["target_yaw"]) {wp.target_yaw = w["target_yaw"].as<double>();}
      out.waypoints.push_back(wp);
    }
  } catch (const std::exception & e) {
    error = std::string("YAML 字段解析失败: ") + e.what();
    return false;
  }

  return true;
}

}  // namespace mission_manager
