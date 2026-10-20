#ifndef ME_NAV2_BRINGUP__RECOVERY_CLEARANCE_HPP_
#define ME_NAV2_BRINGUP__RECOVERY_CLEARANCE_HPP_

#include "escape_trajectory.hpp"

namespace me_nav2_bringup
{
namespace escape
{
struct ScanPoint {double x, y;};
struct Clearance
{
  double half_length{0.345}, half_width{0.285}, margin{0.025};
  double min_linear{0.02}, min_angular{0.04}, horizon{0.5};
  int min_points{2};
};

inline double rectangleDistance(const ScanPoint & point, const Pose & pose, const Clearance & cfg)
{
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  const double x = point.x - pose.x, y = point.y - pose.y;
  const double dx = std::fabs(c * x + s * y) - cfg.half_length;
  const double dy = std::fabs(-s * x + c * y) - cfg.half_width;
  return std::hypot(std::max(dx, 0.0), std::max(dy, 0.0)) +
    std::min(std::max(dx, dy), 0.0);
}

// Same clearance rule as recovery_motion.py; checks every pose in a candidate.
inline bool scanSafe(
  const std::vector<Pose> & poses, const std::vector<ScanPoint> & points, const Clearance & cfg)
{
  if (poses.empty() || points.empty()) {return false;}
  double displacement = 0.0;
  const double radius = std::hypot(cfg.half_length, cfg.half_width);
  for (const auto & pose : poses) {
    displacement = std::max(displacement,
      std::hypot(pose.x - poses.front().x, pose.y - poses.front().y) +
      radius * std::fabs(wrap(pose.yaw - poses.front().yaw)));
  }
  int blocked = 0;
  for (const auto & point : points) {
    const double initial = rectangleDistance(point, poses.front(), cfg);
    if (initial > displacement + cfg.margin) {continue;}
    const double required = std::min(initial, cfg.margin);
    bool hit = initial <= 0.0;
    for (const auto & pose : poses) {
      const double distance = rectangleDistance(point, pose, cfg);
      if (distance <= 0.0 || distance < required - 1e-6) {hit = true; break;}
    }
    if (hit && ++blocked >= cfg.min_points) {return false;}
  }
  return true;
}

inline Motion limitByScan(
  const Pose & start, double v, double w, const std::vector<ScanPoint> & points,
  const Clearance & cfg, const Options & opt,
  const std::function<bool(const std::vector<Pose> &)> & extra_check = {})
{
  for (double ratio : {1.0, 0.75, 0.5, 0.25, 0.125}) {
    Motion cmd;
    cmd.v = v * ratio; cmd.w = w * ratio; cmd.duration = cfg.horizon;
    if (ratio < 1.0 && ((std::fabs(v) > 1e-3 && std::fabs(cmd.v) < cfg.min_linear) ||
      (std::fabs(v) <= 1e-3 && std::fabs(cmd.w) < cfg.min_angular))) {continue;}
    cmd.poses = rollout(start, cmd.v, cmd.w, cmd.duration, opt);
    if (scanSafe(cmd.poses, points, cfg) && (!extra_check || extra_check(cmd.poses))) {
      return cmd;
    }
  }
  return {};
}
}  // namespace escape
}  // namespace me_nav2_bringup
#endif  // ME_NAV2_BRINGUP__RECOVERY_CLEARANCE_HPP_
