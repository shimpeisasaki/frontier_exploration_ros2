#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2
{

struct DirectionSwitchConfig
{
  double stall_timeout_s{15.0};
  double progress_distance_m{0.5};
  double avoidance_duration_s{90.0};
  double locality_radius_m{3.0};
};

// Small, local exploration memory. It never changes a map or motion permission.
class DirectionSwitch
{
public:
  using XY = std::pair<double, double>;
  explicit DirectionSwitch(DirectionSwitchConfig config = {});
  void begin_goal(XY position, XY target, int64_t now_ns);
  bool observe(XY position, XY target, int64_t now_ns);
  std::vector<std::size_t> select(const std::vector<XY> & targets, XY position, int64_t now_ns);
  void reset();
  void suspend_progress() {progress_.reset();}
  std::size_t record_count() const {return records_.size();}

private:
  struct Progress {XY position; int64_t since_ns;};
  struct Avoidance {XY origin; XY direction; int64_t since_ns;};
  void prune(XY position, int64_t now_ns);
  DirectionSwitchConfig config_;
  std::optional<Progress> progress_;
  std::optional<XY> last_goal_target_;
  std::vector<Avoidance> records_;
  std::optional<int64_t> last_time_ns_;
};

}  // namespace frontier_exploration_ros2
