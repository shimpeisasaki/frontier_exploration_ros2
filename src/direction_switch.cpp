#include "frontier_exploration_ros2/direction_switch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace frontier_exploration_ros2
{
namespace
{
bool finite(DirectionSwitch::XY p) {return std::isfinite(p.first) && std::isfinite(p.second);}
double distance(DirectionSwitch::XY a, DirectionSwitch::XY b)
{
  return std::hypot(a.first - b.first, a.second - b.second);
}
}  // namespace

DirectionSwitch::DirectionSwitch(DirectionSwitchConfig config) : config_(config)
{
  if (!std::isfinite(config.stall_timeout_s) || config.stall_timeout_s < 0.25 ||
    !std::isfinite(config.progress_distance_m) || config.progress_distance_m < 0.05 ||
    !std::isfinite(config.avoidance_duration_s) || config.avoidance_duration_s <= 0 ||
    !std::isfinite(config.locality_radius_m) || config.locality_radius_m < config.progress_distance_m)
  {
    throw std::invalid_argument("Invalid direction-switch configuration");
  }
  records_.reserve(4);
}

void DirectionSwitch::reset()
{
  progress_.reset();
  last_goal_target_.reset();
  records_.clear();
  last_time_ns_.reset();
}

void DirectionSwitch::prune(XY position, int64_t now_ns)
{
  if (last_time_ns_ && now_ns < *last_time_ns_) {
    reset();
  }
  last_time_ns_ = now_ns;
  const bool had_records = !records_.empty();
  records_.erase(std::remove_if(records_.begin(), records_.end(), [&](const auto & record) {
      return (now_ns - record.since_ns) / 1e9 >= config_.avoidance_duration_s ||
             distance(position, record.origin) >= config_.locality_radius_m;
    }), records_.end());
  if (had_records && records_.empty()) {
    progress_.reset();  // A newly permitted retry gets a fresh trial window.
  }
}

void DirectionSwitch::begin_goal(XY position, XY target, int64_t now_ns)
{
  if (!finite(position) || !finite(target)) {
    suspend_progress();
    return;
  }
  prune(position, now_ns);
  if (last_goal_target_) {
    const double old_length = distance(position, *last_goal_target_);
    const double new_length = distance(position, target);
    if (old_length > .05 && new_length > .05) {
      const double cosine =
        ((last_goal_target_->first-position.first)*(target.first-position.first) +
        (last_goal_target_->second-position.second)*(target.second-position.second)) /
        (old_length*new_length);
      if (cosine < .5) {  // A genuinely different (>60 deg) goal must get time to move.
        progress_ = Progress{position, now_ns};
      }
    }
  }
  last_goal_target_ = target;
}

bool DirectionSwitch::observe(XY position, XY target, int64_t now_ns)
{
  if (!finite(position) || !finite(target)) {
    suspend_progress();
    return false;
  }
  prune(position, now_ns);
  if (!progress_ || distance(position, progress_->position) >= config_.progress_distance_m) {
    progress_ = Progress{position, now_ns};
    return false;
  }
  if ((now_ns - progress_->since_ns) / 1e9 < config_.stall_timeout_s) {
    return false;
  }
  const double length = distance(position, target);
  if (length < 0.05) {
    progress_ = Progress{position, now_ns};
    return false;
  }
  if (records_.size() == 4) {
    records_.erase(records_.begin());
  }
  records_.push_back(Avoidance{position,
    {(target.first - position.first) / length, (target.second - position.second) / length}, now_ns});
  progress_ = Progress{position, now_ns};
  return true;
}

std::vector<std::size_t> DirectionSwitch::select(
  const std::vector<XY> & targets, XY position, int64_t now_ns)
{
  if (!finite(position)) {
    return {};
  }
  prune(position, now_ns);
  std::vector<std::size_t> allowed, opposite;
  std::vector<double> cosines(targets.size(), 1.0);
  double best_cosine = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < targets.size(); ++i) {
    if (records_.empty()) {
      allowed.push_back(i);
      continue;
    }
    const double length = distance(position, targets[i]);
    if (!finite(targets[i]) || length < 0.05) {
      continue;
    }
    const XY unit{(targets[i].first - position.first) / length,
      (targets[i].second - position.second) / length};
    bool blocked = false;
    for (const auto & record : records_) {
      if (unit.first * record.direction.first + unit.second * record.direction.second >= 0.5 - 1e-9) {
        blocked = true;  // Exclude +/-60 degrees around every locally stalled direction.
        break;
      }
    }
    if (blocked) {
      continue;
    }
    const auto & direction = records_.back().direction;
    cosines[i] = unit.first * direction.first + unit.second * direction.second;
    best_cosine = std::min(best_cosine, cosines[i]);
    allowed.push_back(i);
    if (cosines[i] <= -0.5 + 1e-9) {  // Prefer >=120 degrees from the latest stalled target.
      opposite.push_back(i);
    }
  }
  if (records_.empty() || !opposite.empty()) {
    return records_.empty() ? allowed : opposite;
  }
  // No opposite frontier: keep the greatest change in direction, stable on ties.
  allowed.erase(std::remove_if(allowed.begin(), allowed.end(), [&](std::size_t i) {
      return cosines[i] > best_cosine + 1e-9;
    }), allowed.end());
  return allowed;
}

}  // namespace frontier_exploration_ros2
