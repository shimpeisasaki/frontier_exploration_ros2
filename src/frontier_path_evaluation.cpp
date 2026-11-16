// Optional bounded Nav2 evaluation at dispatch time. No work on sensor callbacks.
#include "frontier_exploration_ros2/frontier_explorer_node.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{

void FrontierExplorerNode::cancelPathEvaluation()
{
  auto evaluation = std::move(path_evaluation_);
  if (!evaluation) {
    return;
  }
  if (evaluation->handle) {
    path_client_->async_cancel_goal(evaluation->handle);
  }
  core_->cancel_path_evaluation(evaluation->request.dispatch_id);
}

void FrontierExplorerNode::dispatchGoalRequest(const GoalDispatchRequest & request)
{
  cancelPathEvaluation();
  if (!path_client_ || request.goal_kind != "frontier") {
    sendNavigateGoal(request);
    return;
  }
  const auto pose = getCurrentPose();
  auto evaluation = std::make_shared<PathEvaluation>();
  evaluation->request = request;
  path_evaluation_ = evaluation;
  if (pose) {
    const auto count = std::min(
      request.frontier_sequence.size(), static_cast<std::size_t>(params_.path_evaluation_candidate_limit));
    for (std::size_t i = 0; i < count; ++i) {
      evaluation->goals.push_back(i == 0 ? request.goal_pose :
        core_->build_dispatch_goal_pose(request.frontier_sequence[i], *pose));
    }
  }
  evaluation->lengths.assign(evaluation->goals.size(), -2.0);
  evaluateNextPath();
}

void FrontierExplorerNode::evaluateNextPath()
{
  auto evaluation = path_evaluation_;
  if (!evaluation) {
    return;
  }
  if (evaluation->index == evaluation->goals.size()) {
    path_evaluation_.reset();
    const auto selected = core_->apply_path_evaluation(
      evaluation->request, evaluation->goals, evaluation->lengths);
    if (selected) {
      sendNavigateGoal(*selected);
    }
    return;
  }
  evaluation->handle.reset();
  evaluation->deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(path_evaluation_timeout_s_));
  if (!path_client_->action_server_is_ready()) {
    // Keep the whole dispatch pending until its deadline, without unverified fallback.
    return;
  }
  const auto index = evaluation->index;
  ComputePath::Goal goal;
  goal.goal = evaluation->goals[index];
  goal.goal.header.stamp = now();
  goal.planner_id = "GridBased";
  rclcpp_action::Client<ComputePath>::SendGoalOptions options;
  options.goal_response_callback = [this, evaluation, index](PathGoalHandle::SharedPtr handle) {
      if (path_evaluation_ != evaluation || evaluation->index != index) {
        if (handle) {
          path_client_->async_cancel_goal(handle);
        }
        return;
      }
      if (!handle) {
        // Rejection is an unavailable planner, not proof of an unreachable frontier.
        evaluation->index++;
        evaluateNextPath();
        return;
      }
      evaluation->handle = handle;
    };
  options.result_callback = [this, evaluation, index](const PathGoalHandle::WrappedResult & result) {
      if (path_evaluation_ != evaluation || evaluation->index != index) {
        return;
      }
      double length = -2.0;
      if (result.code == rclcpp_action::ResultCode::ABORTED) {
        length = -1.0;
      } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result) {
        const auto & path = result.result->path;
        const auto & target = evaluation->goals[index];
        bool valid = path.header.frame_id == target.header.frame_id && !path.poses.empty();
        length = 0.0;
        for (std::size_t i = 0; i < path.poses.size(); ++i) {
          const auto & p = path.poses[i].pose.position;
          valid = valid && std::isfinite(p.x) && std::isfinite(p.y);
          if (i > 0) {
            const auto & previous = path.poses[i - 1].pose.position;
            length += std::hypot(p.x - previous.x, p.y - previous.y);
          }
        }
        if (!valid || !std::isfinite(length)) {
          length = -2.0;
        } else {
          const auto & end = path.poses.back().pose.position;
          if (std::hypot(end.x - target.pose.position.x, end.y - target.pose.position.y) > 0.15) {
            // Navfn may return a nearby endpoint under its goal tolerance.
            length = -1.0;
          }
        }
      }
      evaluation->lengths[index] = length;
      evaluation->index++;
      evaluateNextPath();
    };
  try {
    path_client_->async_send_goal(goal, options);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(get_logger(), "Frontier path evaluation unavailable: %s", exc.what());
    evaluation->index++;
    evaluateNextPath();
  }
}

void FrontierExplorerNode::pathEvaluationTick()
{
  auto evaluation = path_evaluation_;
  if (!evaluation) {
    return;
  }
  if (!core_->exploration_enabled || core_->current_dispatch_id != evaluation->request.dispatch_id ||
    core_->pending_cancel_reason.has_value())
  {
    cancelPathEvaluation();
    return;
  }
  if (std::chrono::steady_clock::now() >= evaluation->deadline) {
    if (evaluation->handle) {
      path_client_->async_cancel_goal(evaluation->handle);
    }
    evaluation->index++;
    evaluateNextPath();
  }
}

}  // namespace frontier_exploration_ros2
