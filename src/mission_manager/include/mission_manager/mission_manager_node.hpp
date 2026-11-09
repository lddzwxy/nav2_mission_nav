#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mission_manager/mission_state_machine.hpp"
#include "mission_manager/waypoint.hpp"
#include "mission_msgs/msg/mission_status.hpp"
#include "mission_msgs/srv/mission_command.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace mission_manager
{

class MissionManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MissionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- 服务与话题 ---
  void OnCommand(
    const std::shared_ptr<mission_msgs::srv::MissionCommand::Request> req,
    std::shared_ptr<mission_msgs::srv::MissionCommand::Response> res);
  void HandleCommand(const std::string & command, const std::string & mission_file,
    bool & ok, std::string & message);

  // --- 主循环：驱动状态机 ---
  void Tick();

  // --- 各状态处理 ---
  void StartNavigation();
  void StartArrivedAction();
  void FinishWaypoint();
  void RetryOrFail();
  void PublishStatus();
  void PublishWaypointMarkers();

  // --- action 回调 ---
  void OnGoalResponse(const NavigateGoalHandle::SharedPtr & handle);
  void OnFeedback(
    const NavigateGoalHandle::SharedPtr & handle,
    const std::shared_ptr<const NavigateToPose::Feedback> & feedback);
  void OnResult(const NavigateGoalHandle::WrappedResult & result);

  // --- 工具 ---
  geometry_msgs::msg::PoseStamped MakePose(const Waypoint & wp) const;
  bool LoadMission(const std::string & path, std::string & error);
  void SaveProgress();
  std::optional<size_t> LoadProgress();

  // --- 数据 ---
  std::mutex mutex_;
  MissionConfig mission_;
  MissionStateMachine sm_;
  std::string mission_file_;
  std::string default_mission_file_;

  size_t current_index_{0};
  int retry_count_{0};
  size_t completed_count_{0};
  std::string last_error_;
  std::chrono::steady_clock::time_point nav_start_time_;
  std::chrono::steady_clock::time_point action_deadline_;
  std::chrono::steady_clock::time_point action_start_time_;

  NavigateGoalHandle::SharedPtr active_goal_;
  bool goal_inflight_{false};
  bool goal_succeeded_{false};
  bool goal_failed_{false};
  std::string goal_error_;
  double last_feedback_distance_{0.0};

  // --- ROS 接口 ---
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Service<mission_msgs::srv::MissionCommand>::SharedPtr cmd_srv_;
  rclcpp::Publisher<mission_msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace mission_manager
