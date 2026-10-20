#ifndef EXPLORE_NAVIGATION_FAILURE_GUARD_H_
#define EXPLORE_NAVIGATION_FAILURE_GUARD_H_

#include <cmath>
#include <stdexcept>

namespace explore
{
// Distinguish a failed destination from repeated failures at the robot's origin.
// Translation is measured in odom; spinning and map-frame jumps are not progress.
class NavigationFailureGuard
{
public:
  explicit NavigationFailureGuard(double distance = 0.05, int limit = 2)
  : distance_(distance), limit_(limit)
  {
    if (!std::isfinite(distance) || distance <= 0 || limit < 1) {
      throw std::invalid_argument("Invalid immobile navigation failure parameters");
    }
  }

  void begin(bool valid, double x, double y)
  {
    if (valid) {observe(x, y);}
    goal_valid_ = valid;
    goal_x_ = x; goal_y_ = y;
    goal_moved_ = false;
  }

  void observe(double x, double y)
  {
    if (!anchor_valid_) {
      anchor_x_ = x; anchor_y_ = y; anchor_valid_ = true;
    }
    if (std::hypot(x - anchor_x_, y - anchor_y_) >= distance_) {
      failures_ = 0; unresolved_ = false;
      anchor_x_ = x; anchor_y_ = y;
    }
    if (goal_valid_ && std::hypot(x - goal_x_, y - goal_y_) >= distance_) {
      goal_moved_ = true;
    }
  }

  bool failed(bool valid, double x, double y, bool origin_blocked)
  {
    if (valid) {observe(x, y);}
    if (valid && goal_valid_ && goal_moved_) {
      failures_ = 0; unresolved_ = false;
      return false;
    }
    unresolved_ = true;
    ++failures_;
    return !valid || !goal_valid_ || origin_blocked || failures_ >= limit_;
  }

  bool unresolved() const {return unresolved_;}
  void reset()
  {
    failures_ = 0; unresolved_ = goal_valid_ = goal_moved_ = anchor_valid_ = false;
  }

private:
  double distance_, goal_x_{0}, goal_y_{0}, anchor_x_{0}, anchor_y_{0};
  int limit_, failures_{0};
  bool unresolved_{false}, goal_valid_{false}, goal_moved_{false}, anchor_valid_{false};
};
}  // namespace explore
#endif  // EXPLORE_NAVIGATION_FAILURE_GUARD_H_
