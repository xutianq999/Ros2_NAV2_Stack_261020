#include "smart_recovery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace me_nav2_bringup
{
using nav2_behaviors::Status;

void SmartRecovery::onConfigure()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("SmartRecovery: node is unavailable");}
  auto param = [&](const std::string & key, double fallback) {
      const std::string name = behavior_name_ + "." + key;
      nav2_util::declare_parameter_if_not_declared(node, name, rclcpp::ParameterValue(fallback));
      const double value = node->get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(name + " must be finite and positive");
      }
      return value;
    };
  backup_distance_ = param("backup_distance", 0.5);
  backup_speed_ = param("backup_speed", 0.1);
  options_.max_length = param("segment_length", 0.4);
  options_.max_curvature = param("max_curvature", 1.5);
  options_.max_yaw = param("max_segment_yaw", 0.52);
  options_.sample_step = param("collision_step", 0.005);
  clearance_.half_length = param("half_length", 0.345);
  clearance_.half_width = param("half_width", 0.285);
  clearance_.margin = param("min_clearance", 0.025);
  clearance_.min_linear = param("min_linear", 0.02);
  clearance_.min_angular = param("min_angular", 0.04);
  clearance_.horizon = param("lookahead_time", 0.5);
  scan_timeout_ = param("scan_timeout", 0.5);
  options_.footprint_radius = param("footprint_radius", 0.46);
  options_.rotate_speed = param("rotate_speed", 0.3);
  stall_timeout_ = param("stall_timeout", 2.0);
  execute_distance_ = param("execute_distance", 0.1);
  yaw_gain_ = param("yaw_gain", 1.5);
  cross_track_gain_ = param("cross_track_gain", 2.0);
  max_cross_track_ = param("max_cross_track", 0.06);
  max_yaw_error_ = param("max_yaw_error", 0.26);
  max_rotation_ = param("max_total_rotation", 0.78);
  min_exit_distance_ = param("min_exit_distance", 0.05);
  exit_cost_ = param("exit_cost", 180.0);
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".max_replans", rclcpp::ParameterValue(4));
  node->get_parameter(behavior_name_ + ".enabled", smart_enabled_);
  node->get_parameter(behavior_name_ + ".max_replans", max_replans_);
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".min_points", rclcpp::ParameterValue(2));
  node->get_parameter(behavior_name_ + ".min_points", clearance_.min_points);
  if (max_replans_ < 0 || backup_speed_ > 0.15 || options_.rotate_speed > 0.8 ||
    options_.max_yaw > 0.7854 ||
    options_.max_length > 0.5 || options_.sample_step > 0.005 ||
    clearance_.margin < 0.01 || clearance_.min_points < 1 || clearance_.horizon < 0.3 ||
    options_.footprint_radius < std::hypot(clearance_.half_length, clearance_.half_width))
  {
    throw std::invalid_argument("SmartRecovery: parameters exceed short-motion safety limits");
  }
  nav2_util::declare_parameter_if_not_declared(
    node, "local_frame", rclcpp::ParameterValue(std::string("odom")));
  node->get_parameter("local_frame", local_frame_);

  // Free-space escape parameters.
  free_space_max_distance_ = param("free_space_max_distance", 2.5);
  free_space_costmap_timeout_ = param("free_space_costmap_timeout", 1.0);
  free_space_trigger_cost_ = param("free_space_trigger_cost", 254.0);
  free_space_target_cost_ = param("free_space_target_cost", 252.0);
  free_space_search_radius_ = param("free_space_search_radius", 5.0);
  free_space_lookahead_ = param("free_space_lookahead", 0.35);
  free_space_blocked_penalty_ = param("free_space_blocked_penalty", 20.0);
  free_space_max_total_rotation_ = param("free_space_max_total_rotation", 3.14);
  max_time_allowance_ = param("max_time_allowance", 40.0);
  auto declare_bool = [&](const std::string & key, bool fallback) {
      const std::string name = behavior_name_ + "." + key;
      nav2_util::declare_parameter_if_not_declared(
        node, name, rclcpp::ParameterValue(fallback));
      return node->get_parameter(name).as_bool();
    };
  auto declare_string = [&](const std::string & key, const std::string & fallback) {
      const std::string name = behavior_name_ + "." + key;
      nav2_util::declare_parameter_if_not_declared(
        node, name, rclcpp::ParameterValue(fallback));
      return node->get_parameter(name).as_string();
    };
  free_space_escape_ = declare_bool("free_space_escape", true);
  free_space_allow_blocked_ = declare_bool("free_space_allow_blocked", true);
  free_space_allow_unknown_ = declare_bool("free_space_allow_unknown", false);
  free_space_costmap_topic_ = declare_string(
    "free_space_costmap_topic", "/global_costmap/costmap_raw");
  free_space_costmap_frame_ = declare_string("free_space_costmap_frame", "map");
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".free_space_max_cells",
    rclcpp::ParameterValue(static_cast<int64_t>(50000)));
  const int64_t max_cells = node->get_parameter(
    behavior_name_ + ".free_space_max_cells").as_int();
  if (max_cells <= 0) {
    throw std::invalid_argument("free_space_max_cells must be positive");
  }
  free_space_max_cells_ = static_cast<size_t>(max_cells);
  if (free_space_target_cost_ >= 253.0 ||
    free_space_trigger_cost_ < free_space_target_cost_ ||
    free_space_trigger_cost_ > 255.0 ||
    free_space_lookahead_ > options_.max_length ||
    free_space_lookahead_ < 0.04)
  {
    throw std::invalid_argument("SmartRecovery: invalid free-space escape limits");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".scan_topic", rclcpp::ParameterValue(std::string("/scan")));
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    node->get_parameter(behavior_name_ + ".scan_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&SmartRecovery::onScan, this, std::placeholders::_1));
  free_costmap_stamp_ = clock_->now();
  if (free_space_escape_) {
    free_costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      free_space_costmap_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav2_msgs::msg::Costmap::SharedPtr msg) {onFreeCostmap(msg);});
  }
  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/recovery_path", 1);
  candidates_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/recovery_candidates", 1);
  history_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&SmartRecovery::rememberPose, this));
  RCLCPP_INFO(logger_, "SmartRecovery: short reverse/forward arcs, full sweep and motion feedback");
}

void SmartRecovery::activate()
{
  nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>::activate();
  path_pub_->on_activate();
  candidates_pub_->on_activate();
}

void SmartRecovery::deactivate()
{
  running_ = false;
  clearPlan();
  path_pub_->on_deactivate();
  candidates_pub_->on_deactivate();
  nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>::deactivate();
}

void SmartRecovery::onCleanup()
{
  running_ = false;
  history_timer_.reset();
  scan_sub_.reset();
  free_costmap_sub_.reset();
  path_mode_ = false;
  free_path_.clear();
  {
    std::lock_guard<std::mutex> lock(free_costmap_mutex_);
    free_costmap_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_points_.clear();
  }
  path_pub_.reset();
  candidates_pub_.reset();
  std::lock_guard<std::mutex> lock(history_mutex_);
  history_.clear();
}

void SmartRecovery::onActionCompletion()
{
  running_ = false;
  path_mode_ = false;
  free_path_.clear();
  clearPlan();
}

bool SmartRecovery::getPose(escape::Pose & out)
{
  geometry_msgs::msg::PoseStamped pose;
  if (!nav2_util::getCurrentPose(
      pose, *tf_, local_frame_, robot_base_frame_, transform_tolerance_)) {return false;}
  out = {pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)};
  return true;
}

void SmartRecovery::onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (!std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) ||
    !std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
    msg->angle_increment <= 0.0 || msg->range_max <= msg->range_min) {return;}
  const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
  if (!std::isfinite(age) || age < -0.1 || age > scan_timeout_) {return;}
  std::vector<escape::ScanPoint> points;
  try {
    const auto transform = tf_->lookupTransform(
      local_frame_, msg->header.frame_id, tf2::TimePointZero);
    const double yaw = tf2::getYaw(transform.transform.rotation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r <= std::max<double>(msg->range_min, 0.02) ||
        r >= msg->range_max) {continue;}
      const double angle = msg->angle_min + i * msg->angle_increment;
      const double x = r * std::cos(angle), y = r * std::sin(angle);
      points.push_back({c * x - s * y + transform.transform.translation.x,
        s * x + c * y + transform.transform.translation.y});
    }
  } catch (const tf2::TransformException &) {return;}
  // Empty or failed scans cannot refresh the last valid observation.
  if (points.empty()) {return;}
  std::lock_guard<std::mutex> lock(scan_mutex_);
  scan_points_ = std::move(points);
  scan_time_ = std::chrono::steady_clock::now();
}

bool SmartRecovery::scanSnapshot(std::vector<escape::ScanPoint> & points)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (scan_points_.empty() || std::chrono::duration<double>(
      std::chrono::steady_clock::now() - scan_time_).count() > scan_timeout_) {return false;}
  points = scan_points_;
  return true;
}

double SmartRecovery::poseCost(const escape::Pose & pose, bool fetch)
{
  geometry_msgs::msg::Pose2D p;
  p.x = pose.x; p.y = pose.y; p.theta = pose.yaw;
  return collision_checker_->scorePose(p, fetch);
}

void SmartRecovery::onFreeCostmap(nav2_msgs::msg::Costmap::SharedPtr msg)
{
  if (!msg) {return;}
  const auto & meta = msg->metadata;
  if (meta.size_x == 0 || meta.size_y == 0 || meta.resolution <= 0.0 ||
    msg->data.size() < static_cast<size_t>(meta.size_x) * meta.size_y)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(free_costmap_mutex_);
  free_costmap_ = msg;
  free_costmap_stamp_ = clock_->now();
}

bool SmartRecovery::costmapTransform(
  const std::string & frame, double & tx, double & ty, double & yaw)
{
  try {
    const auto transform = tf_->lookupTransform(frame, local_frame_, tf2::TimePointZero);
    tx = transform.transform.translation.x;
    ty = transform.transform.translation.y;
    yaw = tf2::getYaw(transform.transform.rotation);
    return std::isfinite(tx) && std::isfinite(ty) && std::isfinite(yaw);
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool SmartRecovery::globalCost(const escape::Pose & pose, double & cost)
{
  nav2_msgs::msg::Costmap::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(free_costmap_mutex_);
    if (!free_costmap_) {return false;}
    if ((clock_->now() - free_costmap_stamp_).seconds() > free_space_costmap_timeout_) {
      return false;
    }
    msg = free_costmap_;
  }
  const auto & meta = msg->metadata;
  if (meta.resolution <= 0.0 || meta.size_x == 0 || meta.size_y == 0) {return false;}
  const std::string frame = msg->header.frame_id.empty() ?
    free_space_costmap_frame_ : msg->header.frame_id;
  double tx = 0.0, ty = 0.0, yaw = 0.0;
  if (!costmapTransform(frame, tx, ty, yaw)) {return false;}
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double wx = c * pose.x - s * pose.y + tx;
  const double wy = s * pose.x + c * pose.y + ty;
  const double gx = (wx - meta.origin.position.x) / meta.resolution;
  const double gy = (wy - meta.origin.position.y) / meta.resolution;
  if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0.0 || gy < 0.0) {return false;}
  const unsigned int mx = static_cast<unsigned int>(gx);
  const unsigned int my = static_cast<unsigned int>(gy);
  if (mx >= meta.size_x || my >= meta.size_y) {return false;}
  cost = static_cast<double>(msg->data[static_cast<size_t>(my) * meta.size_x + mx]);
  return true;
}

bool SmartRecovery::validateFreePath(
  const escape::Pose & pose, std::vector<escape::Pose> & path,
  const std::vector<escape::ScanPoint> & scan)
{
  (void)pose;
  if (path.size() < 2) {return false;}

  // The global costmap may be wrong about static walls, so the local costmap
  // and the live scan remain the hard safety gate. Truncate before the first
  // local lethal cell.
  bool fetch = true;
  size_t limit = path.size();
  try {
    for (size_t i = 0; i < path.size(); ++i) {
      const double cost = poseCost(path[i], fetch);
      fetch = false;
      if (!std::isfinite(cost) || cost < 0.0) {return false;}
      if (cost >= 254.0) {limit = i; break;}
    }
  } catch (const std::exception &) {
    return false;
  }
  if (limit < 2) {return false;}
  path.resize(limit);

  // If the tail is not scan-safe, keep the longest safe prefix.
  static const double fractions[] = {1.0, 0.75, 0.5, 0.35, 0.2, 0.1};
  for (double fraction : fractions) {
    size_t count = static_cast<size_t>(std::ceil(path.size() * fraction));
    count = std::max<size_t>(2, std::min(count, path.size()));
    std::vector<escape::Pose> prefix(path.begin(), path.begin() + count);
    if (!escape::scanSafe(prefix, scan, clearance_)) {continue;}
    double length = 0.0;
    for (size_t i = 1; i < prefix.size(); ++i) {
      length += std::hypot(
        prefix[i].x - prefix[i - 1].x, prefix[i].y - prefix[i - 1].y);
    }
    if (length + 1e-6 < min_exit_distance_) {return false;}
    path.swap(prefix);
    return true;
  }
  return false;
}

bool SmartRecovery::selectGuidedMotion(const escape::Pose & pose)
{
  std::vector<escape::ScanPoint> scan;
  if (!scanSnapshot(scan)) {return false;}
  auto opt = options_;
  opt.speed = speed_;
  const double remaining = free_space_max_distance_ - travelled_;
  if (remaining <= 0.0) {return false;}
  opt.max_length = std::min(opt.max_length, remaining);
  opt.allow_rotation =
    rotated_ + 0.26 <= free_space_max_total_rotation_ + 1e-6;

  const double base = std::min(free_space_lookahead_, opt.max_length);
  for (double scale : {1.0, 0.7, 0.45, 0.25}) {
    const double lookahead = base * scale;
    if (lookahead < 0.04) {continue;}
    const auto motion = escape::guidedMotion(
      pose, free_path_, lookahead, speed_, opt);
    if (!motion.valid() || !escape::scanSafe(motion.poses, scan, clearance_)) {
      continue;
    }
    try {
      bool fetch = true;
      const bool safe = escape::safe(motion.poses, [&](escape::Pose p) {
        const double cost = poseCost(p, fetch);
        fetch = false;
        return cost;
      }, true);
      if (!safe) {continue;}
    } catch (const std::exception &) {
      return false;
    }
    motion_ = motion;
    RCLCPP_INFO(
      logger_, "Free-space motion: v=%.2f w=%.2f dur=%.2f%s",
      motion_.v, motion_.w, motion_.duration,
      motion_.rotating() ? " (rotation)" : "");
    segment_progress_ = progress_mark_ = 0.0;
    last_progress_time_ = clock_->now();
    return true;
  }

  if (!opt.allow_rotation) {return false;}
  const auto rotation = escape::guidedRotation(
    pose, free_path_, options_.rotate_speed, opt);
  if (!rotation.valid() || !escape::scanSafe(rotation.poses, scan, clearance_)) {
    return false;
  }
  try {
    bool fetch = true;
    const bool safe = escape::safe(rotation.poses, [&](escape::Pose p) {
      const double cost = poseCost(p, fetch);
      fetch = false;
      return cost;
    }, true);
    if (!safe) {return false;}
  } catch (const std::exception &) {
    return false;
  }
  motion_ = rotation;
  RCLCPP_INFO(
    logger_, "Free-space motion: v=%.2f w=%.2f dur=%.2f (rotation)",
    motion_.v, motion_.w, motion_.duration);
  segment_progress_ = progress_mark_ = 0.0;
  last_progress_time_ = clock_->now();
  return true;
}

bool SmartRecovery::planFromFreeSpace(const escape::Pose & pose)
{
  path_mode_ = false;
  free_path_.clear();
  if (!free_space_escape_) {return false;}

  nav2_msgs::msg::Costmap::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(free_costmap_mutex_);
    if (!free_costmap_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "Free-space escape: waiting for %s", free_space_costmap_topic_.c_str());
      return false;
    }
    if ((clock_->now() - free_costmap_stamp_).seconds() > free_space_costmap_timeout_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "Free-space escape: %s is stale", free_space_costmap_topic_.c_str());
      return false;
    }
    msg = free_costmap_;
  }
  const auto & meta = msg->metadata;
  if (meta.resolution <= 0.0 || meta.size_x == 0 || meta.size_y == 0 ||
    msg->data.size() < static_cast<size_t>(meta.size_x) * meta.size_y)
  {
    return false;
  }
  const std::string frame = msg->header.frame_id.empty() ?
    free_space_costmap_frame_ : msg->header.frame_id;
  double tx = 0.0, ty = 0.0, yaw = 0.0;
  if (!costmapTransform(frame, tx, ty, yaw)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000, "Free-space escape: TF %s -> %s unavailable",
      frame.c_str(), local_frame_.c_str());
    return false;
  }

  const double c = std::cos(yaw), s = std::sin(yaw);
  const double wx = c * pose.x - s * pose.y + tx;
  const double wy = s * pose.x + c * pose.y + ty;
  const double gx = (wx - meta.origin.position.x) / meta.resolution;
  const double gy = (wy - meta.origin.position.y) / meta.resolution;
  if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0.0 || gy < 0.0) {return false;}
  const int sx = static_cast<int>(gx), sy = static_cast<int>(gy);
  if (sx >= static_cast<int>(meta.size_x) ||
    sy >= static_cast<int>(meta.size_y))
  {
    return false;
  }
  const unsigned char start_cost =
    msg->data[static_cast<size_t>(sy) * meta.size_x + sx];
  if (static_cast<double>(start_cost) < free_space_trigger_cost_) {return false;}
  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000,
    "Free-space escape: start cost=%u, searching %s",
    static_cast<unsigned int>(start_cost), free_space_costmap_topic_.c_str());

  const escape::GridView grid{
    msg->data.data(), meta.size_x, meta.size_y, meta.resolution,
    meta.origin.position.x, meta.origin.position.y};
  escape::GridPathOptions opt;
  opt.target_cost = free_space_target_cost_;
  opt.blocked_penalty = free_space_blocked_penalty_;
  opt.allow_blocked = free_space_allow_blocked_;
  opt.allow_unknown = free_space_allow_unknown_;
  opt.max_search_distance = free_space_search_radius_;
  opt.max_cells = free_space_max_cells_;
  const auto grid_path = escape::nearestTraversablePath(grid, sx, sy, opt);
  if (!grid_path.found) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Free-space escape: no traversable cell within %.1f m (start cost=%u)",
      free_space_search_radius_, static_cast<unsigned int>(start_cost));
    return false;
  }

  std::vector<escape::Pose> path;
  path.reserve(grid_path.cells.size());
  for (const auto & cell : grid_path.cells) {
    const double cx = meta.origin.position.x + (cell.x + 0.5) * meta.resolution;
    const double cy = meta.origin.position.y + (cell.y + 0.5) * meta.resolution;
    const double dx = cx - tx, dy = cy - ty;
    path.push_back({c * dx + s * dy, -s * dx + c * dy, 0.0});
  }
  if (path.empty()) {return false;}
  path.front() = pose;

  std::vector<escape::Pose> dense;
  dense.push_back(path.front());
  const double step = 0.02;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];
    const auto & b = path[i];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d = std::hypot(dx, dy);
    if (d < 1e-6) {continue;}
    const double heading = std::atan2(dy, dx);
    const int n = std::max(1, static_cast<int>(std::ceil(d / step)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      dense.push_back({a.x + dx * t, a.y + dy * t, heading});
    }
  }
  if (dense.size() < 2) {return false;}
  dense.front().yaw = dense[1].yaw;

  std::vector<escape::ScanPoint> scan;
  if (!scanSnapshot(scan)) {return false;}
  if (!validateFreePath(pose, dense, scan)) {return false;}

  free_path_ = std::move(dense);
  path_crossed_blocked_ = grid_path.crossed_blocked;
  path_mode_ = true;
  if (!selectGuidedMotion(pose)) {
    path_mode_ = false;
    free_path_.clear();
    return false;
  }
  publishPlan({});

  const auto & target = grid_path.cells.back();
  RCLCPP_INFO(
    logger_,
    "Free-space escape: start cost=%u, path %.2f m, target cost=%u%s",
    static_cast<unsigned int>(start_cost), grid_path.length,
    static_cast<unsigned int>(grid.at(target.x, target.y)),
    grid_path.crossed_blocked ? ", crossed blocked cells" : "");
  return true;
}

void SmartRecovery::rememberPose()
{
  if (running_ || !tf_->canTransform(local_frame_, robot_base_frame_, tf2::TimePointZero)) {return;}
  escape::Pose pose;
  if (!getPose(pose)) {return;}
  const double now = clock_->now().seconds();
  std::lock_guard<std::mutex> lock(history_mutex_);
  if (!history_.empty()) {
    const auto & last = history_.back();
    const double d = std::hypot(pose.x - last.second.x, pose.y - last.second.y);
    if (now < last.first || now - last.first > 30.0 || d > 0.5) {history_.clear();}
    else if (d < 0.04) {return;}
  }
  history_.emplace_back(now, pose);
  while (history_.size() > 75 || now - history_.front().first > 30.0) {history_.pop_front();}
}

bool SmartRecovery::planFrom(const escape::Pose & pose)
{
  if (planFromFreeSpace(pose)) {return true;}
  path_mode_ = false;
  free_path_.clear();
  return planPrimitives(pose);
}

bool SmartRecovery::planPrimitives(const escape::Pose & pose)
{
  std::vector<escape::ScanPoint> scan;
  if (!scanSnapshot(scan)) {
    RCLCPP_WARN(logger_, "SmartRecovery: no fresh scan; stopping");
    return false;
  }
  auto opt = options_;
  opt.speed = speed_;
  const double remaining = distance_ - travelled_;
  if (remaining <= 0.01) {
    RCLCPP_WARN(logger_, "SmartRecovery: distance budget exhausted");
    return false;
  }
  opt.max_length = std::min(opt.max_length, remaining);
  opt.allow_rotation = rotated_ + 0.26 <= max_rotation_ + 1e-6;
  std::vector<escape::Pose> history;
  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    for (const auto & item : history_) {
      if (clock_->now().seconds() - item.first <= 30.0) {history.push_back(item.second);}
    }
  }
  bool fetch = true;
  std::vector<escape::Motion> candidates;
  try {
    motion_ = escape::choose(pose, opt, [&](escape::Pose p) {
        const double cost = poseCost(p, fetch);
        fetch = false;
        return cost;
      }, history, rejected_, &candidates, [&](const std::vector<escape::Pose> & poses) {
        return escape::scanSafe(poses, scan, clearance_);
      });
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "SmartRecovery: collision data unavailable: %s", e.what());
    motion_ = {};
  }
  if (!motion_.valid()) {
    RCLCPP_WARN(logger_, "SmartRecovery: no checked escape primitive; stopping");
    clearPlan();
    return false;
  }
  segment_progress_ = progress_mark_ = 0.0;
  last_progress_time_ = clock_->now();
  publishPlan(candidates);
  RCLCPP_INFO(
    logger_, "SmartRecovery: v=%.2f w=%.2f, length=%.2f m, turn=%.0f deg%s",
    motion_.v, motion_.w, std::fabs(motion_.v) * motion_.duration,
    motion_.w * motion_.duration * 180.0 / M_PI, motion_.rotating() ? " (checked rotation)" : "");
  return true;
}

Status SmartRecovery::onRun(const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> goal)
{
  if (!smart_enabled_ || !std::isfinite(goal->target.x) || !std::isfinite(goal->speed) ||
    !getPose(action_start_)) {return finish(Status::FAILED);}
  distance_ = std::min(backup_distance_, std::fabs(goal->target.x) > 0.01 ?
      std::fabs(goal->target.x) : backup_distance_);
  speed_ = std::min(backup_speed_, goal->speed > 0.0 ? goal->speed : backup_speed_);
  const double requested = rclcpp::Duration(goal->time_allowance).seconds();
  allowance_ = requested > 0.0 ?
    std::min(requested, max_time_allowance_) : max_time_allowance_;
  travelled_ = rotated_ = 0.0;
  path_mode_ = false;
  path_crossed_blocked_ = false;
  free_path_.clear();
  replan_count_ = 0;
  rejected_.clear();
  start_time_ = clock_->now();
  running_ = true;
  if (!planFrom(action_start_)) {return finish(Status::FAILED);}
  return Status::SUCCEEDED;
}

Status SmartRecovery::finish(Status status)
{
  stopRobot();
  running_ = false;
  clearPlan();
  return status;
}

Status SmartRecovery::retry(const escape::Pose & pose, const char * reason)
{
  stopRobot();
  rejected_.push_back(motion_);
  RCLCPP_WARN(logger_, "SmartRecovery: %s; excluding this motion", reason);
  if (++replan_count_ > max_replans_ || !planFrom(pose)) {return finish(Status::FAILED);}
  return Status::RUNNING;
}

Status SmartRecovery::onCycleUpdate()
{
  if ((clock_->now() - start_time_).seconds() >= allowance_) {return finish(Status::FAILED);}
  escape::Pose pose;
  if (!getPose(pose)) {return finish(Status::FAILED);}
  if (path_mode_) {
    double cost = 0.0;
    if (globalCost(pose, cost) && cost <= free_space_target_cost_) {
      const double net = std::hypot(
        pose.x - action_start_.x, pose.y - action_start_.y);
      if (net + 0.005 >= min_exit_distance_) {return finish(Status::SUCCEEDED);}
    }
    if (travelled_ >= free_space_max_distance_ - 0.01) {
      RCLCPP_WARN(logger_, "SmartRecovery: free-space escape distance exhausted");
      return finish(Status::FAILED);
    }
  }
  auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
  double progress;
  bool segment_done;
  if (motion_.rotating()) {
    const double delta = escape::wrap(pose.yaw - motion_.poses.front().yaw);
    progress = std::max(0.0, std::copysign(1.0, motion_.w) * delta);
    const double remaining = std::fabs(motion_.w) * motion_.duration - progress;
    segment_done = remaining < 0.025;
    cmd->angular.z = std::copysign(std::min(options_.rotate_speed, 1.5 * remaining), motion_.w);
    rotated_ += std::max(0.0, progress - segment_progress_);
  } else {
    const auto track = escape::tracking(motion_, pose);
    if (std::fabs(track.cross_track) > max_cross_track_ ||
      std::fabs(track.yaw_error) > max_yaw_error_)
    {
      return retry(pose, "tracking error exceeds bounds");
    }
    progress = track.progress;
    travelled_ += std::max(0.0, progress - segment_progress_);
    segment_done = progress >= std::min(execute_distance_,
        std::fabs(motion_.v) * motion_.duration) - 0.005;
    cmd->linear.x = motion_.v;
    const double limit = options_.max_curvature * std::fabs(motion_.v);
    cmd->angular.z = std::clamp(motion_.w + yaw_gain_ * track.yaw_error -
        std::copysign(1.0, motion_.v) * cross_track_gain_ * track.cross_track, -limit, limit);
  }
  segment_progress_ = std::max(segment_progress_, progress);
  if (progress >= progress_mark_ + (motion_.rotating() ? 0.02 : 0.01)) {
    progress_mark_ = progress;
    last_progress_time_ = clock_->now();
  } else if ((clock_->now() - last_progress_time_).seconds() >= stall_timeout_) {
    return retry(pose, "no actual progress");
  }
  auto feedback = std::make_shared<nav2_msgs::action::BackUp::Feedback>();
  feedback->distance_traveled = travelled_;
  action_server_->publish_feedback(feedback);
  if (!path_mode_ && travelled_ >= distance_ - 0.01) {
    return finish(Status::SUCCEEDED);
  }
  if (segment_done) {
    if (path_mode_) {
      if (planFromFreeSpace(pose)) {
        stopRobot();
        return Status::RUNNING;
      }
      path_mode_ = false;
      free_path_.clear();
      if (!planPrimitives(pose)) {return finish(Status::FAILED);}
      stopRobot();
      return Status::RUNNING;
    }
    try {
      const double net = std::hypot(pose.x - action_start_.x, pose.y - action_start_.y);
      const double cost = poseCost(pose, true);
      if (net + 0.005 >= min_exit_distance_ && cost >= 0.0 && cost <= exit_cost_) {
        return finish(Status::SUCCEEDED);
      }
    } catch (const std::exception &) {return finish(Status::FAILED);}
    if (!planFrom(pose)) {return finish(Status::FAILED);}
    // Recompute the command from the newly selected path on the next cycle.
    stopRobot();
    return Status::RUNNING;
  }
  // Check the actual corrected command from the current pose. Never probe an
  // old ideal heading while executing a different one after skid/pose drift.
  try {
    std::vector<escape::ScanPoint> scan;
    if (!scanSnapshot(scan)) {
      // A brief scan gap should not burn a BT retry. Hold zero and let the
      // stall timer / action timeout decide; planFrom() still refuses to pick
      // a new primitive without a fresh scan.
      stopRobot();
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000, "SmartRecovery: scan stale; holding zero");
      return Status::RUNNING;
    }
    const auto checked = escape::limitByScan(
      pose, cmd->linear.x, cmd->angular.z, scan, clearance_, options_,
      [&](const std::vector<escape::Pose> & swept) {
        bool fetch = true;
        return escape::safe(swept, [&](escape::Pose p) {
          const double cost = poseCost(p, fetch);
          fetch = false;
          return cost;
        }, true);
      });
    if (!checked.valid()) {return retry(pose, "corrected command has no safe creeping speed");}
    cmd->linear.x = checked.v;
    cmd->angular.z = checked.w;
  } catch (const std::exception &) {return finish(Status::FAILED);}
  vel_pub_->publish(std::move(cmd));
  return Status::RUNNING;
}

void SmartRecovery::publishPlan(const std::vector<escape::Motion> & candidates)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = local_frame_;
  path.header.stamp = clock_->now();
  const auto & display = path_mode_ ? free_path_ : motion_.poses;
  for (const auto & p : display) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = p.x; pose.pose.position.y = p.y;
    pose.pose.orientation.z = std::sin(p.yaw / 2.0);
    pose.pose.orientation.w = std::cos(p.yaw / 2.0);
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);
  int id = 0;
  for (const auto & candidate : candidates) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "checked_escape_candidates";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.b = 1.0; marker.color.g = 0.6; marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(3.0);
    for (const auto & p : candidate.poses) {
      geometry_msgs::msg::Point point;
      point.x = p.x; point.y = p.y; point.z = 0.04;
      marker.points.push_back(point);
    }
    array.markers.push_back(marker);
  }
  if (path_mode_ && !free_path_.empty()) {
    visualization_msgs::msg::Marker target;
    target.header = path.header;
    target.ns = "free_space_target";
    target.id = 0;
    target.type = visualization_msgs::msg::Marker::SPHERE;
    target.action = visualization_msgs::msg::Marker::ADD;
    target.pose.position.x = free_path_.back().x;
    target.pose.position.y = free_path_.back().y;
    target.pose.position.z = 0.08;
    target.pose.orientation.w = 1.0;
    target.scale.x = target.scale.y = target.scale.z = 0.12;
    target.color.g = 1.0; target.color.a = 0.85;
    target.lifetime = rclcpp::Duration::from_seconds(3.0);
    array.markers.push_back(target);
  }
  candidates_pub_->publish(array);
}

void SmartRecovery::clearPlan()
{
  if (path_pub_ && path_pub_->is_activated()) {
    nav_msgs::msg::Path path;
    path.header.frame_id = local_frame_;
    path.header.stamp = clock_->now();
    path_pub_->publish(path);
  }
  if (candidates_pub_ && candidates_pub_->is_activated()) {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    candidates_pub_->publish(array);
  }
}
}  // namespace me_nav2_bringup

PLUGINLIB_EXPORT_CLASS(me_nav2_bringup::SmartRecovery, nav2_core::Behavior)
