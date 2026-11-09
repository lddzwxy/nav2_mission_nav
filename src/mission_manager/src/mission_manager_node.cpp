#include "mission_manager/mission_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace mission_manager
{

namespace
{
constexpr char kDefaultMissionFile[] = "";
constexpr char kProgressFilePath[] = "/tmp/mission_progress.yaml";
}  // namespace

MissionManagerNode::MissionManagerNode(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  default_mission_file_ = declare_parameter<std::string>("mission_file", kDefaultMissionFile);
  const auto tick_rate = declare_parameter<double>("tick_rate", 5.0);

  sm_.SetSkipOnFail(true);
  sm_.SetTransitionHook(
    [this](State from, Event ev, State to) {
      RCLCPP_INFO(get_logger(), "状态机: %s --%d--> %s",
        ToString(from), static_cast<int>(ev), ToString(to));
    });

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  status_pub_ = create_publisher<mission_msgs::msg::MissionStatus>("/mission_status", 10);
  progress_pub_ = create_publisher<std_msgs::msg::Float32>("/mission_progress", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mission_waypoints_vis", 10);

  cmd_srv_ = create_service<mission_msgs::srv::MissionCommand>(
    "/mission_command",
    std::bind(&MissionManagerNode::OnCommand, this, std::placeholders::_1, std::placeholders::_2));

  tick_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / tick_rate),
    std::bind(&MissionManagerNode::Tick, this));

  // 启动即加载默认任务（若有）
  if (!default_mission_file_.empty()) {
    std::string err;
    if (LoadMission(default_mission_file_, err)) {
      sm_.Handle(Event::kLoaded);
      RCLCPP_INFO(get_logger(), "默认任务 %s 已加载，%zu 个路点",
        mission_.name.c_str(), mission_.waypoints.size());
    } else {
      RCLCPP_ERROR(get_logger(), "默认任务加载失败: %s", err.c_str());
      last_error_ = err;
      sm_.Handle(Event::kLoadFailed);
    }
  } else {
    RCLCPP_INFO(get_logger(), "未配置 mission_file，等待 /mission_command START 指定");
  }
}

bool MissionManagerNode::LoadMission(const std::string & path, std::string & error)
{
  MissionConfig cfg;
  if (!LoadMissionFromYaml(path, cfg, error)) {
    return false;
  }
  mission_ = std::move(cfg);
  mission_file_ = path;
  sm_.SetSkipOnFail(mission_.skip_on_fail);
  current_index_ = 0;
  retry_count_ = 0;
  completed_count_ = 0;
  PublishWaypointMarkers();
  return true;
}

void MissionManagerNode::OnCommand(
  const std::shared_ptr<mission_msgs::srv::MissionCommand::Request> req,
  std::shared_ptr<mission_msgs::srv::MissionCommand::Response> res)
{
  HandleCommand(req->command, req->mission_file, res->success, res->message);
  RCLCPP_INFO(get_logger(), "命令 %s -> %s (%s)",
    req->command.c_str(), res->success ? "OK" : "FAIL", res->message.c_str());
}

void MissionManagerNode::HandleCommand(
  const std::string & command, const std::string & mission_file, bool & ok, std::string & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ok = true;
  message = "";

  if (command == "START") {
    std::string file = mission_file.empty() ? mission_file_ : mission_file;
    if (!file.empty() && file != mission_file_) {
      std::string err;
      if (!LoadMission(file, err)) {
        ok = false;
        message = err;
        return;
      }
    }
    if (mission_.waypoints.empty()) {
      ok = false;
      message = "无可用路点，先 RELOAD 或 START 时指定 mission_file";
      return;
    }
    // 断点续跑：存在进度文件则从上次路点继续
    if (auto saved = LoadProgress(); saved.has_value() && *saved < mission_.waypoints.size()) {
      current_index_ = *saved;
      completed_count_ = *saved;
      RCLCPP_INFO(get_logger(), "断点续跑：从路点 %zu (%s) 继续",
        current_index_, mission_.waypoints[current_index_].id.c_str());
    }
    if (!sm_.Handle(Event::kStartCmd)) {
      ok = false;
      message = std::string("当前状态 ") + ToString(sm_.state()) + " 不接受 START";
    }
    return;
  }

  if (command == "PAUSE") {
    if (!sm_.Handle(Event::kPauseCmd)) {
      ok = false;
      message = std::string("当前状态 ") + ToString(sm_.state()) + " 不接受 PAUSE";
    } else if (goal_inflight_) {
      nav_client_->async_cancel_goal(active_goal_);
      goal_inflight_ = false;
    }
    return;
  }

  if (command == "RESUME") {
    if (!sm_.Handle(Event::kResumeCmd)) {
      ok = false;
      message = std::string("当前状态 ") + ToString(sm_.state()) + " 不接受 RESUME";
    }
    return;
  }

  if (command == "ABORT") {
    if (!sm_.Handle(Event::kAbortCmd)) {
      ok = false;
      message = std::string("当前状态 ") + ToString(sm_.state()) + " 不接受 ABORT";
    } else if (goal_inflight_) {
      nav_client_->async_cancel_goal(active_goal_);
      goal_inflight_ = false;
    }
    return;
  }

  if (command == "RELOAD") {
    std::string file = mission_file.empty() ? mission_file_ : mission_file;
    if (file.empty()) {
      ok = false;
      message = "无 mission_file 可加载";
      return;
    }
    std::string err;
    if (!LoadMission(file, err)) {
      ok = false;
      message = err;
    } else {
      // 终态（DONE/FAILED/ABORTED）经 kLoaded 回 IDLE
      sm_.Handle(Event::kLoaded);
    }
    return;
  }

  ok = false;
  message = "未知命令: " + command;
}

void MissionManagerNode::Tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // action 结果轮询
  if (goal_succeeded_) {
    goal_succeeded_ = false;
    goal_inflight_ = false;
    if (sm_.Handle(Event::kNavSucceeded)) {
      StartArrivedAction();
    }
  } else if (goal_failed_) {
    goal_failed_ = false;
    goal_inflight_ = false;
    last_error_ = goal_error_;
    sm_.Handle(Event::kNavFailed);
  }

  // 导航超时保护（goal_timeout_sec）
  if (sm_.state() == State::kNavigating && goal_inflight_) {
    const auto elapsed = std::chrono::steady_clock::now() - nav_start_time_;
    if (std::chrono::duration<double>(elapsed).count() > mission_.goal_timeout_sec) {
      RCLCPP_WARN(get_logger(), "导航超时 %.1fs，取消目标", mission_.goal_timeout_sec);
      nav_client_->async_cancel_goal(active_goal_);
      goal_inflight_ = false;
      goal_failed_ = true;
      goal_error_ = "goal timeout";
      return;
    }
  }

  // RETRYING：决定重试还是跳过/失败
  if (sm_.state() == State::kRetrying) {
    RetryOrFail();
    return;
  }

  // DO_ACTION：wait / spin 计时
  if (sm_.state() == State::kDoAction) {
    if (std::chrono::steady_clock::now() >= action_deadline_) {
      FinishWaypoint();
    }
    return;
  }

  // NAVIGATING 但没有在飞的 goal → 发目标（暂停恢复/重试/新目标统一入口）
  if (sm_.state() == State::kNavigating && !goal_inflight_) {
    StartNavigation();
    return;
  }

  // SKIPPED：推进
  if (sm_.state() == State::kSkipped) {
    FinishWaypoint();
    return;
  }

  PublishStatus();
}

void MissionManagerNode::StartNavigation()
{
  if (current_index_ >= mission_.waypoints.size()) {
    sm_.Handle(Event::kAllWaypointsDone);
    PublishStatus();
    return;
  }

  const auto & wp = mission_.waypoints[current_index_];
  if (!nav_client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "navigate_to_pose 服务未就绪，等待 Nav2 ...");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = MakePose(wp);

  RCLCPP_INFO(get_logger(), "导航 → [%zu/%zu] %s (%.2f, %.2f, %.2f)",
    current_index_ + 1, mission_.waypoints.size(), wp.id.c_str(), wp.x, wp.y, wp.yaw);

  nav_start_time_ = std::chrono::steady_clock::now();
  goal_inflight_ = true;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
  opts.goal_response_callback =
    std::bind(&MissionManagerNode::OnGoalResponse, this, std::placeholders::_1);
  opts.feedback_callback =
    std::bind(&MissionManagerNode::OnFeedback, this, std::placeholders::_1, std::placeholders::_2);
  opts.result_callback =
    std::bind(&MissionManagerNode::OnResult, this, std::placeholders::_1);
  nav_client_->async_send_goal(goal, opts);
}

void MissionManagerNode::StartArrivedAction()
{
  const auto & wp = mission_.waypoints[current_index_];
  retry_count_ = 0;

  switch (wp.action) {
    case WaypointAction::kWait:
      RCLCPP_INFO(get_logger(), "到点 %s，等待 %.1fs", wp.id.c_str(), wp.duration_sec);
      action_start_time_ = std::chrono::steady_clock::now();
      action_deadline_ = action_start_time_ +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(wp.duration_sec));
      sm_.Handle(Event::kActionStart);   // ARRIVED → DO_ACTION，Tick 里等 deadline
      return;
    case WaypointAction::kSpin:
      RCLCPP_INFO(get_logger(), "到点 %s，原地旋转 %.2f rad", wp.id.c_str(), wp.target_yaw);
      action_start_time_ = std::chrono::steady_clock::now();
      // 简化实现：按 0.8 rad/s 估时停留（真机可换 nav2 Spin action）
      action_deadline_ = action_start_time_ +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::fabs(wp.target_yaw) / 0.8 + 1.0));
      sm_.Handle(Event::kActionStart);
      return;
    case WaypointAction::kCustom:
      RCLCPP_WARN(get_logger(), "路点 %s action=custom 未注册 handler，直接完成", wp.id.c_str());
      [[fallthrough]];
    case WaypointAction::kNavigate:
      // 无子任务：直接推进
      FinishWaypoint();
      return;
  }
}

void MissionManagerNode::FinishWaypoint()
{
  completed_count_ = current_index_ + 1;
  SaveProgress();
  current_index_++;

  if (current_index_ >= mission_.waypoints.size()) {
    if (mission_.loop && !mission_.waypoints.empty()) {
      RCLCPP_INFO(get_logger(), "一轮完成，loop=true 重新开始");
      current_index_ = 0;
      completed_count_ = 0;
      // kActionDone 从 ARRIVED/DO_ACTION/SKIPPED 都能进 NAVIGATING
      sm_.Handle(Event::kActionDone);
    } else {
      sm_.Handle(Event::kAllWaypointsDone);
      RCLCPP_INFO(get_logger(), "任务 %s 全部完成", mission_.name.c_str());
    }
  } else {
    sm_.Handle(Event::kActionDone);
  }
  PublishStatus();
}

void MissionManagerNode::RetryOrFail()
{
  retry_count_++;
  if (retry_count_ <= mission_.max_retries) {
    RCLCPP_WARN(get_logger(), "路点 %s 失败（%s），第 %d 次重试",
      mission_.waypoints[current_index_].id.c_str(), last_error_.c_str(), retry_count_);
    sm_.Handle(Event::kRetryOk);
    return;
  }

  RCLCPP_ERROR(get_logger(), "路点 %s 重试用尽（max_retries=%d）",
    mission_.waypoints[current_index_].id.c_str(), mission_.max_retries);
  if (mission_.skip_on_fail) {
    RCLCPP_WARN(get_logger(), "skip_on_fail=true，跳过该路点继续");
    sm_.Handle(Event::kRetryExhausted);
    // SKIPPED 态在下个 Tick 由 FinishWaypoint 推进
  } else {
    sm_.Handle(Event::kRetryExhausted);  // → FAILED
    PublishStatus();
  }
}

void MissionManagerNode::OnGoalResponse(const NavigateGoalHandle::SharedPtr & handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!handle) {
    goal_inflight_ = false;
    goal_failed_ = true;
    goal_error_ = "goal rejected";
  } else {
    active_goal_ = handle;
  }
}

void MissionManagerNode::OnFeedback(
  const NavigateGoalHandle::SharedPtr & /*handle*/,
  const std::shared_ptr<const NavigateToPose::Feedback> & feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_feedback_distance_ = feedback->distance_remaining;
}

void MissionManagerNode::OnResult(const NavigateGoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  goal_inflight_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      goal_succeeded_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      goal_failed_ = true;
      goal_error_ = "aborted: " + std::string(
        result.result ? std::to_string(result.result->error_code) : "?");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      goal_failed_ = true;
      goal_error_ = "canceled";
      break;
    default:
      goal_failed_ = true;
      goal_error_ = "unknown result";
      break;
  }
}

geometry_msgs::msg::PoseStamped MissionManagerNode::MakePose(const Waypoint & wp) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = now();
  pose.pose.position.x = wp.x;
  pose.pose.position.y = wp.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
  pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
  return pose;
}

void MissionManagerNode::PublishStatus()
{
  mission_msgs::msg::MissionStatus msg;
  msg.header.stamp = now();
  msg.mission_name = mission_.name;
  msg.state = static_cast<uint8_t>(sm_.state());
  msg.current_index = static_cast<uint8_t>(
    std::min(current_index_, static_cast<size_t>(255)));
  msg.total_waypoints = static_cast<uint8_t>(
    std::min(mission_.waypoints.size(), static_cast<size_t>(255)));
  msg.current_wp_id =
    current_index_ < mission_.waypoints.size() ?
    mission_.waypoints[current_index_].id : "";
  msg.retry_count = static_cast<uint8_t>(std::min(retry_count_, 255));
  msg.progress = mission_.waypoints.empty() ? 0.0F :
    static_cast<float>(completed_count_) / static_cast<float>(mission_.waypoints.size());
  msg.last_error = last_error_;
  msg.eta_hint.sec = 0;
  msg.eta_hint.nanosec = 0;
  status_pub_->publish(msg);

  std_msgs::msg::Float32 progress;
  progress.data = msg.progress;
  progress_pub_->publish(progress);
}

void MissionManagerNode::PublishWaypointMarkers()
{
  visualization_msgs::msg::MarkerArray arr;
  // 清除旧标记
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  for (size_t i = 0; i < mission_.waypoints.size(); ++i) {
    const auto & wp = mission_.waypoints[i];
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = "mission_waypoints";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = MakePose(wp).pose;
    m.scale.x = 0.35;
    m.scale.y = 0.08;
    m.scale.z = 0.08;
    m.color.r = 0.1F;
    m.color.g = 0.7F;
    m.color.b = 1.0F;
    m.color.a = 0.9F;
    arr.markers.push_back(m);

    visualization_msgs::msg::Marker text = m;
    text.id = static_cast<int>(i) + 1000;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.pose.position.z = 0.35;
    text.scale.z = 0.18;
    text.color.r = 1.0F;
    text.color.g = 1.0F;
    text.color.b = 1.0F;
    text.text = wp.id;
    arr.markers.push_back(text);
  }
  marker_pub_->publish(arr);
}

void MissionManagerNode::SaveProgress()
{
  std::ofstream ofs(kProgressFilePath);
  if (ofs) {
    ofs << "index: " << current_index_ << "\n"
        << "mission_file: " << mission_file_ << "\n";
  }
}

std::optional<size_t> MissionManagerNode::LoadProgress()
{
  std::ifstream ifs(kProgressFilePath);
  if (!ifs) {
    return std::nullopt;
  }
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.rfind("index: ", 0) == 0) {
      return static_cast<size_t>(std::stoul(line.substr(7)));
    }
  }
  return std::nullopt;
}

}  // namespace mission_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
