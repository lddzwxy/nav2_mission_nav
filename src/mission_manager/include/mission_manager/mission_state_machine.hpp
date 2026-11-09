#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace mission_manager
{

// 与 mission_msgs/MissionStatus 的 STATE_* 常量一一对应
enum class State : uint8_t
{
  kLoading = 0,
  kIdle = 1,
  kNavigating = 2,
  kArrived = 3,
  kDoAction = 4,
  kRetrying = 5,
  kSkipped = 6,
  kPaused = 7,
  kDone = 8,
  kFailed = 9,
  kAborted = 10,
};

const char * ToString(State state);

enum class Event
{
  kLoaded,          // YAML 加载成功
  kLoadFailed,      // YAML 格式错误
  kStartCmd,
  kPauseCmd,
  kResumeCmd,
  kAbortCmd,
  kNavSucceeded,
  kNavFailed,       // action 失败/超时/取消
  kRetryOk,         // 重试预算未用尽
  kRetryExhausted,  // 重试用尽
  kActionStart,     // 到点后进入 wait/spin/custom 子任务
  kActionDone,      // 到点子任务完成
  kAllWaypointsDone,
};

// 纯逻辑状态机：不碰 ROS，便于单测与讲解
class MissionStateMachine
{
public:
  using TransitionHook = std::function<void (State from, Event ev, State to)>;

  State state() const {return state_;}
  void SetTransitionHook(TransitionHook hook) {hook_ = std::move(hook);}

  // 事件入口，非法事件返回 false（状态不变）
  bool Handle(Event ev);

  // 重试计数由节点维护并经参数传入，状态机只做去留判断
  void SetSkipOnFail(bool skip) {skip_on_fail_ = skip;}

private:
  void Transit(Event ev, State to);

  State state_{State::kLoading};
  bool skip_on_fail_{true};
  TransitionHook hook_;
};

}  // namespace mission_manager
