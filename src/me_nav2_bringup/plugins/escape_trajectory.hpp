#ifndef ME_NAV2_BRINGUP__ESCAPE_TRAJECTORY_HPP_
#define ME_NAV2_BRINGUP__ESCAPE_TRAJECTORY_HPP_

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace me_nav2_bringup
{
namespace escape
{
struct Pose {double x{0.0}, y{0.0}, yaw{0.0};};
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

// Exact constant-twist integration; v is signed, including reverse arcs.
inline Pose integrate(Pose p, double v, double w, double t)
{
  if (std::fabs(w) < 1e-8) {
    p.x += v * t * std::cos(p.yaw);
    p.y += v * t * std::sin(p.yaw);
  } else {
    p.x += v / w * (std::sin(p.yaw + w * t) - std::sin(p.yaw));
    p.y -= v / w * (std::cos(p.yaw + w * t) - std::cos(p.yaw));
  }
  p.yaw = wrap(p.yaw + w * t);
  return p;
}

struct Motion
{
  double v{0.0}, w{0.0}, duration{0.0};
  std::vector<Pose> poses;
  double score{std::numeric_limits<double>::infinity()};
  bool valid() const {return !poses.empty();}
  bool rotating() const {return std::fabs(v) < 1e-6;}
};

struct Options
{
  double speed{0.1};
  double max_length{0.4};
  double max_curvature{1.5};
  double max_yaw{0.52};
  double sample_step{0.005};
  double footprint_radius{0.46};
  double rotate_speed{0.3};
  bool allow_rotation{true};
};

inline std::vector<Pose> rollout(
  const Pose & start, double v, double w, double duration, const Options & opt)
{
  // Bound the displacement of the farthest footprint point, not just its center.
  const double sweep = (std::fabs(v) + opt.footprint_radius * std::fabs(w)) * duration;
  const int n = std::max(1, static_cast<int>(std::ceil(sweep / opt.sample_step)));
  std::vector<Pose> poses;
  poses.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {
    poses.push_back(integrate(start, v, w, duration * i / n));
  }
  return poses;
}

inline bool safe(
  const std::vector<Pose> & poses, const std::function<double(Pose)> & cost,
  bool allow_blocked_start = false)
{
  if (poses.empty()) {return true;}
  const double start_cost = cost(poses.front());
  // A negative or non-finite score means the observation is unusable, not
  // "already in collision": there is nothing trustworthy to escape from.
  if (!std::isfinite(start_cost) || start_cost < 0.0) {return false;}
  if (start_cost >= 254.0 && !allow_blocked_start) {return false;}
  for (size_t i = 1; i < poses.size(); ++i) {
    const double c = cost(poses[i]);
    if (!std::isfinite(c) || c < 0.0) {return false;}
    // Already in collision: moving is allowed, getting worse is not.
    if (start_cost >= 254.0) {
      // Unknown (255) is not "worse" than lethal (254) numerically, but a
      // definite lethal cell is still an obstacle: never enter one from
      // unknown space.
      if (start_cost > 254.0 && c >= 254.0 && c < 255.0) {return false;}
      if (c > start_cost + 1e-9) {return false;}
    } else if (c >= 254.0) {
      return false;
    }
  }
  return true;
}

inline bool rejected(const Motion & candidate, const std::vector<Motion> & failures)
{
  for (const auto & failed : failures) {
    if (candidate.rotating() != failed.rotating()) {continue;}
    if (candidate.rotating()) {
      if (candidate.w * failed.w > 0.0) {return true;}
    } else if (candidate.v * failed.v > 0.0 &&
      std::fabs(candidate.w / candidate.v - failed.w / failed.v) < 0.2)
    {
      return true;
    }
  }
  return false;
}

inline Motion choose(
  const Pose & start, const Options & opt, const std::function<double(Pose)> & cost,
  const std::vector<Pose> & breadcrumbs, const std::vector<Motion> & failures,
  std::vector<Motion> * candidates = nullptr,
  const std::function<bool(const std::vector<Pose> &)> & scan_safe = {})
{
  // A robot that is already inside lethal/unknown cost still needs a way out.
  // Allow the starting pose to be blocked, but never allow the trajectory to
  // make the collision score worse. Non-finite/negative scores stay fatal:
  // they mean there is no usable costmap observation at all.
  if (!std::isfinite(opt.max_length) || opt.max_length <= 0.0 ||
    !std::isfinite(opt.speed) || opt.speed <= 0.0)
  {
    return Motion{};
  }
  const double start_cost = cost(start);
  if (!std::isfinite(start_cost) || start_cost < 0.0) {return Motion{};}
  const bool start_blocked = start_cost >= 254.0;

  Motion best;
  auto consider = [&](Motion motion) {
      if (rejected(motion, failures)) {return;}
      motion.poses = rollout(start, motion.v, motion.w, motion.duration, opt);
      if (scan_safe && !scan_safe(motion.poses)) {return;}
      double total = start_cost;
      double terminal = start_cost;
      // The starting footprint was checked once above; check every later
      // sampled pose, including the full rotation sweep.
      for (size_t i = 1; i < motion.poses.size(); ++i) {
        const double c = cost(motion.poses[i]);
        if (!std::isfinite(c) || c < 0.0) {return;}
        if (start_blocked) {
          // Unknown start may move within unknown or into free space, but a
          // lethal (254) cell remains a hard obstacle.
          if (start_cost > 254.0 && c >= 254.0 && c < 255.0) {return;}
          if (c > start_cost + 1e-9) {return;}
        } else if (c >= 254.0) {
          return;
        }
        terminal = c;
        total += c;
      }
      const double length = std::fabs(motion.v) * motion.duration;
      const double turn = std::fabs(motion.w) * motion.duration;
      motion.score = 4.0 * terminal / 253.0 +
        2.0 * total / (253.0 * motion.poses.size()) + 1.5 * turn +
        (motion.rotating() ? 0.0 : 0.4 * std::fabs(motion.w / motion.v)) +
        0.5 * (1.0 - length / opt.max_length);
      // History is a preference only. Every historical route is rechecked now.
      if (!breadcrumbs.empty() && motion.v < 0.0) {
        double d = 0.3;
        for (const auto & p : breadcrumbs) {
          d = std::min(d, std::hypot(p.x - motion.poses.back().x, p.y - motion.poses.back().y));
        }
        motion.score += d / 0.3;
      }
      if (candidates) {candidates->push_back(motion);}
      if (motion.score < best.score) {best = motion;}
    };

  // Reverse first; consider forward only when no safe reverse primitive remains.
  for (double sign : {-1.0, 1.0}) {
    std::vector<double> curvatures{0.0, -0.5 * opt.max_curvature,
      0.5 * opt.max_curvature, -opt.max_curvature, opt.max_curvature};
    if (sign < 0.0) {
      for (const auto & p : breadcrumbs) {
        const double dx = p.x - start.x, dy = p.y - start.y;
        const double x = std::cos(start.yaw) * dx + std::sin(start.yaw) * dy;
        const double y = -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
        const double d2 = x * x + y * y;
        if (x < -0.1 && d2 < opt.max_length * opt.max_length && d2 > 0.01) {
          const double curvature = 2.0 * y / d2;
          if (std::fabs(curvature) <= opt.max_curvature) {curvatures.push_back(curvature);}
        }
      }
    }
    for (double fraction : {1.0, 0.625, 0.375, 0.125}) {
      const double length = opt.max_length * fraction;
      for (double curvature : curvatures) {
        if (std::fabs(curvature) * length > opt.max_yaw) {continue;}
        Motion m;
        m.v = sign * opt.speed;
        m.w = m.v * curvature;
        m.duration = length / opt.speed;
        consider(m);
      }
    }
    if (best.valid()) {return best;}
  }
  // Small checked rotations are a last resort, never a blind 90/180-degree spin.
  if (opt.allow_rotation) {
    for (double sign : {-1.0, 1.0}) {
      Motion m;
      m.w = sign * opt.rotate_speed;
      m.duration = std::min(0.26, opt.max_yaw) / opt.rotate_speed;
      consider(m);
    }
  }
  return best;
}

struct Tracking
{
  double progress{0.0}, cross_track{0.0}, yaw_error{0.0};
};

inline Tracking tracking(const Motion & motion, const Pose & pose)
{
  Tracking result;
  double best_distance = std::numeric_limits<double>::infinity();
  double length = 0.0;
  for (size_t i = 1; i < motion.poses.size(); ++i) {
    const auto & a = motion.poses[i - 1];
    const auto & b = motion.poses[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < 1e-12) {continue;}
    const double t = std::clamp(((pose.x - a.x) * dx + (pose.y - a.y) * dy) / d2, 0.0, 1.0);
    const double rx = a.x + t * dx, ry = a.y + t * dy;
    const double yaw = wrap(a.yaw + t * wrap(b.yaw - a.yaw));
    const double dist = std::hypot(pose.x - rx, pose.y - ry);
    if (dist < best_distance) {
      best_distance = dist;
      result.progress = length + t * std::sqrt(d2);
      result.cross_track = -std::sin(yaw) * (pose.x - rx) + std::cos(yaw) * (pose.y - ry);
      result.yaw_error = wrap(yaw - pose.yaw);
    }
    length += std::sqrt(d2);
  }
  return result;
}

// Index of the point on a world-frame path nearest to the robot, plus the
// point roughly `lookahead` metres further along the path.
struct PathPursuit
{
  size_t index{0};
  double distance{0.0};
};

inline PathPursuit pursuePath(
  const std::vector<Pose> & path, const Pose & start, double lookahead)
{
  PathPursuit result;
  if (path.empty()) {return result;}

  size_t closest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].x - start.x, path[i].y - start.y);
    if (d < best) {best = d; closest = i;}
  }
  result.index = closest;

  double travelled = 0.0;
  double px = start.x, py = start.y;
  for (size_t i = closest; i < path.size(); ++i) {
    travelled += std::hypot(path[i].x - px, path[i].y - py);
    px = path[i].x; py = path[i].y;
    result.index = i;
    if (travelled >= lookahead) {break;}
  }
  result.distance = std::hypot(path[result.index].x - start.x,
      path[result.index].y - start.y);
  return result;
}

// Constant-curvature arc from the robot toward a point on a guide path.
// Returns an empty motion when the point is behind, the required curvature
// exceeds the vehicle limit, or the turn would exceed one segment's yaw.
inline Motion guidedMotion(
  const Pose & start, const std::vector<Pose> & path, double lookahead,
  double speed, const Options & opt)
{
  Motion motion;
  if (path.empty() || !std::isfinite(lookahead) || lookahead <= 0.0 ||
    !std::isfinite(speed) || speed <= 0.0)
  {
    return motion;
  }
  const auto pursuit = pursuePath(path, start, lookahead);
  const auto & target = path[pursuit.index];
  const double dx = target.x - start.x, dy = target.y - start.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-3) {return motion;}

  const double c = std::cos(start.yaw), s = std::sin(start.yaw);
  double local_x = c * dx + s * dy;
  double local_y = -s * dx + c * dy;
  // A target behind the robot is pursued in reverse, matching the generic
  // recovery policy; rotating in place toward a 180-degree target is slow.
  const bool reverse = local_x <= 0.0;
  if (reverse) {
    local_x = -local_x;
    local_y = -local_y;
  }
  const double d2 = local_x * local_x + local_y * local_y;
  if (d2 < 1e-6) {return motion;}
  const double curvature = 2.0 * local_y / d2;
  if (std::fabs(curvature) > opt.max_curvature + 1e-9) {return motion;}

  motion.v = reverse ? -speed : speed;
  motion.w = motion.v * curvature;
  motion.duration = distance / speed;
  if (std::fabs(motion.w) * motion.duration > opt.max_yaw + 1e-9) {
    return Motion{};
  }
  motion.poses = rollout(start, motion.v, motion.w, motion.duration, opt);
  return motion;
}

// Checked in-place rotation toward the guide path, used when the next path
// point is behind the robot. The caller still applies the scan/cost checks.
inline Motion guidedRotation(
  const Pose & start, const std::vector<Pose> & path, double rotate_speed,
  const Options & opt)
{
  Motion motion;
  if (path.empty() || !std::isfinite(rotate_speed) || rotate_speed <= 0.0) {
    return motion;
  }
  const auto pursuit = pursuePath(path, start, opt.max_length);
  const auto & target = path[pursuit.index];
  const double bearing = wrap(std::atan2(target.y - start.y, target.x - start.x) - start.yaw);
  if (std::fabs(bearing) < 1e-3) {return motion;}
  motion.w = std::copysign(rotate_speed, bearing);
  motion.duration = std::min(0.26, std::fabs(bearing)) / rotate_speed;
  motion.poses = rollout(start, 0.0, motion.w, motion.duration, opt);
  return motion;
}
}  // namespace escape
}  // namespace me_nav2_bringup

#endif  // ME_NAV2_BRINGUP__ESCAPE_TRAJECTORY_HPP_
