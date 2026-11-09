#include "mission_manager/mission_state_machine.hpp"

namespace mission_manager
{

const char * ToString(State state)
{
  switch (state) {
    case State::kLoading: return "LOADING";
    case State::kIdle: return "IDLE";
    case State::kNavigating: return "NAVIGATING";
    case State::kArrived: return "ARRIVED";
    case State::kDoAction: return "DO_ACTION";
    case State::kRetrying: return "RETRYING";
    case State::kSkipped: return "SKIPPED";
    case State::kPaused: return "PAUSED";
    case State::kDone: return "DONE";
    case State::kFailed: return "FAILED";
    case State::kAborted: return "ABORTED";
  }
  return "UNKNOWN";
}

void MissionStateMachine::Transit(Event ev, State to)
{
  const State from = state_;
  state_ = to;
  if (hook_) {
    hook_(from, ev, to);
  }
}

// 推进语义（由节点择一发事件）：
//   kActionDone       → 本路点完成且还有后续 → 下一个 NAVIGATING
//   kAllWaypointsDone → 本路点完成且已到队尾 → DONE
//   kRetryOk          → 重试预算未尽，重新导航当前路点
//   kRetryExhausted   → 重试用尽：skip_on_fail ? 跳过进 SKIPPED : FAILED
//   SKIPPED 态同样用 kStartCmd / kAllWaypointsDone 推进
bool MissionStateMachine::Handle(Event ev)
{
  // 全局事件
  if (ev == Event::kAbortCmd && state_ != State::kLoading) {
    Transit(ev, State::kAborted);
    return true;
  }
  if (ev == Event::kPauseCmd &&
    (state_ == State::kNavigating || state_ == State::kDoAction || state_ == State::kRetrying))
  {
    Transit(ev, State::kPaused);
    return true;
  }

  switch (state_) {
    case State::kLoading:
      if (ev == Event::kLoaded) {
        Transit(ev, State::kIdle);
        return true;
      }
      if (ev == Event::kLoadFailed) {
        Transit(ev, State::kFailed);
        return true;
      }
      return false;

    case State::kIdle:
      if (ev == Event::kStartCmd) {
        Transit(ev, State::kNavigating);
        return true;
      }
      return false;

    case State::kNavigating:
      if (ev == Event::kNavSucceeded) {
        Transit(ev, State::kArrived);
        return true;
      }
      if (ev == Event::kNavFailed) {
        Transit(ev, State::kRetrying);
        return true;
      }
      return false;

    case State::kArrived:
      // 需要子任务（wait/spin/custom）→ DO_ACTION；navigate 直接推进
      if (ev == Event::kActionStart) {
        Transit(ev, State::kDoAction);
        return true;
      }
      if (ev == Event::kActionDone) {
        Transit(ev, State::kNavigating);
        return true;
      }
      if (ev == Event::kAllWaypointsDone) {
        Transit(ev, State::kDone);
        return true;
      }
      return false;

    case State::kDoAction:
      // wait / spin / custom 子任务完成
      if (ev == Event::kActionDone) {
        Transit(ev, State::kNavigating);
        return true;
      }
      if (ev == Event::kAllWaypointsDone) {
        Transit(ev, State::kDone);
        return true;
      }
      return false;

    case State::kRetrying:
      if (ev == Event::kRetryOk) {
        Transit(ev, State::kNavigating);
        return true;
      }
      if (ev == Event::kRetryExhausted) {
        Transit(ev, skip_on_fail_ ? State::kSkipped : State::kFailed);
        return true;
      }
      return false;

    case State::kSkipped:
      // 跳过当前路点后继续队列（kActionDone 与 kStartCmd 同义，便于 FinishWaypoint 统一推进）
      if (ev == Event::kStartCmd || ev == Event::kActionDone) {
        Transit(ev, State::kNavigating);
        return true;
      }
      if (ev == Event::kAllWaypointsDone) {
        Transit(ev, State::kDone);
        return true;
      }
      return false;

    case State::kPaused:
      if (ev == Event::kResumeCmd) {
        Transit(ev, State::kNavigating);
        return true;
      }
      return false;

    case State::kDone:
    case State::kFailed:
    case State::kAborted:
      // 终态：kLoaded（RELOAD）可回到 IDLE 重新开始；DONE 后循环任务可直接 kStartCmd
      if (ev == Event::kLoaded) {
        Transit(ev, State::kIdle);
        return true;
      }
      if (ev == Event::kStartCmd && state_ == State::kDone) {
        Transit(ev, State::kNavigating);
        return true;
      }
      return false;
  }
  return false;
}

}  // namespace mission_manager
