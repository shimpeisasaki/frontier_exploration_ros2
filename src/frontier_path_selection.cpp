// Keep actual dispatch metadata and failure suppression aligned with path selection.
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::cancel_path_evaluation(int dispatch_id)
{
  if (current_dispatch_id == dispatch_id && goal_state == GoalLifecycleState::SENDING) {
    pending_cancel_reason.reset();
    cancel_request_in_progress = false;
    clear_active_goal_state();
  }
}

std::optional<GoalDispatchRequest> FrontierExplorerCore::apply_path_evaluation(
  const GoalDispatchRequest & request,
  const std::vector<geometry_msgs::msg::PoseStamped> & goals,
  const std::vector<double> & lengths)
{
  if (request.dispatch_id != current_dispatch_id || goal_state != GoalLifecycleState::SENDING ||
    !exploration_enabled || pending_cancel_reason.has_value())
  {
    cancel_path_evaluation(request.dispatch_id);
    return std::nullopt;
  }
  if (goals.size() != lengths.size() || lengths.size() > request.frontier_sequence.size()) {
    cancel_path_evaluation(request.dispatch_id);
    return std::nullopt;
  }

  const auto pose = callbacks.get_current_pose();
  if (!pose) {
    clear_active_goal_state();
    wait_for_next_map_refresh();
    return std::nullopt;
  }
  const auto & q = pose->orientation;
  const RobotState robot{{pose->position.x, pose->position.y},
    std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
  std::vector<std::pair<double, std::size_t>> ranked;
  for (std::size_t i = 0; i < lengths.size(); ++i) {
    const auto & frontier = request.frontier_sequence[i];
    if (lengths[i] == -1.0) {
      record_failed_frontier_attempt(frontier);
    }
    if (!std::isfinite(lengths[i]) || lengths[i] < 0.0 ||
      frontier_cost_status(frontier).has_value())
    {
      continue;
    }
    // Retain the existing gain weighting and translation-time terms, replacing
    // approximate distance with the Nav2 route length. Stable ties keep MRTSP order.
    const double gain = std::pow(std::max(1.0, frontier_information_gain(frontier)), params.weight_gain_ws);
    const auto & target = goals[i].pose.position;
    const double motion_time = lower_bound_time_cost(
      robot, {target.x, target.y}, lengths[i], params.max_linear_speed_vmax, params.max_angular_speed_wmax);
    const double score = params.weight_distance_wd * lengths[i] / gain + motion_time / std::sqrt(gain);
    if (std::isfinite(score)) {
      ranked.emplace_back(score, i);
    }
  }
  std::stable_sort(ranked.begin(), ranked.end(), [](const auto & a, const auto & b) {
      return a.first < b.first;
    });
  if (ranked.empty()) {
    clear_active_goal_state();
    wait_for_next_map_refresh();
    callbacks.log_warn("No evaluated frontier path is usable; waiting before reselection");
    return std::nullopt;
  }

  GoalDispatchRequest selected = request;
  const auto best = ranked.front().second;
  selected.goal_pose = goals[best];
  selected.frontier = request.frontier_sequence[best];
  selected.frontier_sequence.clear();
  for (const auto & entry : ranked) {
    selected.frontier_sequence.push_back(request.frontier_sequence[entry.second]);
  }
  // Keep the unevaluated tail for the existing replacement/suppression policy.
  selected.frontier_sequence.insert(selected.frontier_sequence.end(),
    request.frontier_sequence.begin() + lengths.size(), request.frontier_sequence.end());
  active_goal_pose = selected.goal_pose;
  active_goal_frontier = selected.frontier;
  active_goal_frontiers = selected.frontier_sequence;
  active_goal_sent_time_ns = callbacks.now_ns();
  dispatch_contexts[request.dispatch_id] = DispatchContext{
    request.goal_kind, selected.frontier, selected.frontier_sequence, request.action_name};
  std::ostringstream description;
  description << "Nav2 frontier path selection: candidate=" << best + 1 << "/" << lengths.size()
              << ", path_length_m=" << lengths[best] << ", " << describe_frontier(*selected.frontier);
  selected.description = description.str();
  callbacks.log_info(selected.description);
  if (debug_outputs_enabled()) {
    callbacks.publish_selected_frontier_pose(selected.goal_pose);
  }
  return selected;
}

}  // namespace frontier_exploration_ros2
