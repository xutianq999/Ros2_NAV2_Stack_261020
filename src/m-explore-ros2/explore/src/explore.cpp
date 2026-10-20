/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <algorithm>
#include <thread>

inline static bool same_point(const geometry_msgs::msg::Point& one,
                              const geometry_msgs::msg::Point& two,
                              double tolerance)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < tolerance;
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);
  this->declare_parameter<double>("startup_grace", 10.0);
  this->declare_parameter<double>("goal_hold_distance", 0.25);
  this->declare_parameter<double>("goal_snap_radius", 1.5);
  this->declare_parameter<double>("blacklist_timeout", 120.0);
  this->declare_parameter<bool>("goal_commit", true);
  this->declare_parameter<double>("switch_improvement", 0.3);
  this->declare_parameter<double>("goal_track_radius", 1.5);
  this->declare_parameter<double>("max_gain_size", 2.0);
  this->declare_parameter<std::string>("bounds_topic", "/exploration/bounds");
  this->declare_parameter<std::string>("guide_topic", "/exploration/guide");
  this->declare_parameter<std::string>("guide_clear_topic",
                                       "/exploration/guide_clear");
  this->declare_parameter<double>("guide_timeout", 60.0);
  this->declare_parameter<double>("guide_weight", 2.0);
  this->declare_parameter<double>("guide_cone_deg", 90.0);
  this->declare_parameter<std::string>("guide_mode", "bias");
  this->declare_parameter<bool>("guide_navigation", true);
  this->declare_parameter<double>("guide_min_distance", 0.5);
  this->declare_parameter<double>("goal_lost_grace", 15.0);
  this->declare_parameter<int>("max_goal_attempts", 3);
  this->declare_parameter<double>("giveup_window", 180.0);
  this->declare_parameter<double>("giveup_timeout", 600.0);
  this->declare_parameter<double>("giveup_radius", 1.0);
  this->declare_parameter<double>("no_valid_frontier_grace", 30.0);
  this->declare_parameter<std::string>("progress_frame", "odom");
  this->declare_parameter<double>("immobile_distance", 0.05);
  this->declare_parameter<int>("immobile_failure_limit", 2);
  // auto: start frontier exploration; manual: start paused for manual Nav2.
  this->declare_parameter<std::string>("startup_mode", "auto");

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("startup_grace", startup_grace_);
  this->get_parameter("goal_hold_distance", goal_hold_distance_);
  this->get_parameter("goal_snap_radius", goal_snap_radius_);
  this->get_parameter("blacklist_timeout", blacklist_timeout_);
  this->get_parameter("goal_commit", goal_commit_);
  this->get_parameter("switch_improvement", switch_improvement_);
  this->get_parameter("goal_track_radius", goal_track_radius_);
  this->get_parameter("max_gain_size", max_gain_size_);
  this->get_parameter("guide_timeout", guide_timeout_);
  this->get_parameter("guide_weight", guide_weight_);
  this->get_parameter("guide_cone_deg", guide_cone_deg_);
  this->get_parameter("guide_mode", guide_mode_);
  this->get_parameter("guide_navigation", guide_navigation_);
  this->get_parameter("guide_min_distance", guide_min_distance_);
  this->get_parameter("goal_lost_grace", goal_lost_grace_);
  this->get_parameter("max_goal_attempts", max_goal_attempts_);
  this->get_parameter("giveup_window", giveup_window_);
  this->get_parameter("giveup_timeout", giveup_timeout_);
  this->get_parameter("giveup_radius", giveup_radius_);
  this->get_parameter("no_valid_frontier_grace", no_valid_frontier_grace_);
  this->get_parameter("progress_frame", progress_frame_);
  navigation_failure_guard_ = NavigationFailureGuard(
      this->get_parameter("immobile_distance").as_double(),
      this->get_parameter("immobile_failure_limit").as_int());

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size,
                                                 max_gain_size_, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  recovery_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/safety/recovery_active", 1, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        recovery_active_ = msg->data;
        recovery_seen_ = std::chrono::steady_clock::now();
      });
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  // Visual boundary polygon and direction guide (latched so late subscribers
  // receive the last value).
  const auto bounds_topic = this->get_parameter("bounds_topic").as_string();
  if (!bounds_topic.empty()) {
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    bounds_sub_ =
        this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            bounds_topic, latched,
            std::bind(&Explore::boundsCallback, this, std::placeholders::_1));
  }
  const auto guide_topic = this->get_parameter("guide_topic").as_string();
  if (!guide_topic.empty()) {
    auto latched = rclcpp::QoS(1).reliable().transient_local();
    guide_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        guide_topic, latched,
        std::bind(&Explore::guideCallback, this, std::placeholders::_1));
  }
  const auto guide_clear_topic =
      this->get_parameter("guide_clear_topic").as_string();
  if (!guide_clear_topic.empty()) {
    guide_clear_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        guide_clear_topic, 10,
        std::bind(&Explore::clearGuideCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base nav2 server");

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });

  const std::string startup_mode =
      this->get_parameter("startup_mode").as_string();
  if (startup_mode == "manual" || startup_mode == "false" ||
      startup_mode == "0" || startup_mode == "off") {
    // Deterministic manual startup: do not rely on a separate mode node's
    // message arriving after this subscription is ready.
    RCLCPP_INFO(logger_, "Startup mode is MANUAL; frontier exploration paused.");
    stop();
  } else {
    // Start exploration right away
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
    status_pub_->publish(status_msg);
    makePlan();
  }
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::boundsCallback(
    const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  bounds_.clear();
  bounds_.reserve(msg->polygon.points.size());
  for (const auto& point32 : msg->polygon.points) {
    geometry_msgs::msg::Point point;
    point.x = point32.x;
    point.y = point32.y;
    point.z = point32.z;
    bounds_.push_back(point);
  }
  has_bounds_ = bounds_.size() >= 3;
  frontier_grace_.reset();
  drop_commitment_ = true;  // 用户划了新边界：不要再死守旧目标
  goal_attempts_.clear();   // 用户明确要求的区域：放弃记录作废，重新尝试
  giveup_.clear();
  RCLCPP_INFO(logger_, "Exploration boundary updated: %zu vertices (%s).",
              bounds_.size(), has_bounds_ ? "enabled" : "disabled");

  // Drawing a boundary is an explicit user action: if exploration had stopped
  // (e.g. the previous empty map was considered complete), restart it and give
  // the newly bounded region a fresh grace window.
  if (has_bounds_ && exploring_timer_ && exploring_timer_->is_canceled()) {
    RCLCPP_INFO(logger_,
                "Boundary updated while exploration is stopped; resuming.");
    resume();
  } else if (has_bounds_) {
    // A newly drawn region deserves a fresh grace window: the map may still be
    // growing and initially contain no frontier inside the polygon.
    start_time_ = this->now();
  }
}

void Explore::guideCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg->header.frame_id.empty()) {
    has_guide_ = false;
    frontier_grace_.reset();
    drop_commitment_ = true;
    goal_attempts_.clear();
    giveup_.clear();
    RCLCPP_INFO(logger_, "Direction guide cleared.");
    return;
  }
  guide_ = *msg;
  has_guide_ = true;
  frontier_grace_.reset();
  drop_commitment_ = true;  // 用户给了新方向：立刻按新方向重选
  goal_attempts_.clear();   // 用户明确指的方向：之前“进不去”的放弃记录作废
  giveup_.clear();
  // The user explicitly selected a new target; old blacklist entries must not
  // silently suppress it.
  frontier_blacklist_.clear();
  guide_time_ = this->now();
  RCLCPP_INFO(logger_, "Direction guide set: target=(%.2f, %.2f) frame=%s.",
              msg->pose.position.x, msg->pose.position.y,
              msg->header.frame_id.c_str());

  if (exploring_timer_ && exploring_timer_->is_canceled()) {
    RCLCPP_INFO(logger_, "Guide set while exploration is stopped; resuming.");
    resume();
  } else {
    start_time_ = this->now();
  }
}

void Explore::clearGuideCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  has_guide_ = false;
  frontier_grace_.reset();
  drop_commitment_ = true;
  RCLCPP_INFO(logger_, "Direction guide cleared.");
}

bool Explore::pointInPolygon(const geometry_msgs::msg::Point& point) const
{
  const size_t count = bounds_.size();
  if (count < 3) {
    return false;
  }
  bool inside = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++) {
    const auto& pi = bounds_[i];
    const auto& pj = bounds_[j];
    const bool crosses =
        ((pi.y > point.y) != (pj.y > point.y)) &&
        (point.x < (pj.x - pi.x) * (point.y - pi.y) /
                       (pj.y - pi.y + 1e-12) + pi.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

void Explore::applyBoundsFilter(
    std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::msg::Pose& robot_pose)
{
  (void)robot_pose;  // Robot-out-of-bounds is checked in makePlan() before calling this
  if (!has_bounds_ || bounds_.size() < 3) {
    return;
  }
  const size_t before = frontiers.size();
  frontiers.erase(
      std::remove_if(
          frontiers.begin(), frontiers.end(),
          [this](const frontier_exploration::Frontier& frontier) {
            return !pointInPolygon(frontier.centroid);
          }),
      frontiers.end());
  RCLCPP_DEBUG(logger_, "Bounds filter: %zu -> %zu frontiers.", before,
               frontiers.size());
}

void Explore::applyGuide(
    std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::msg::Pose& robot_pose)
{
  if (!has_guide_) {
    return;
  }
  if (guide_timeout_ > 0.0 &&
      (this->now() - guide_time_).seconds() > guide_timeout_) {
    has_guide_ = false;
    RCLCPP_INFO(logger_, "Direction guide expired.");
    return;
  }

  const double dx = guide_.pose.position.x - robot_pose.position.x;
  const double dy = guide_.pose.position.y - robot_pose.position.y;
  const double distance = std::hypot(dx, dy);
  double guide_yaw = 0.0;
  if (distance < 0.5) {
    // Clicked too close to the robot: use the pose orientation instead.
    const auto& q = guide_.pose.orientation;
    guide_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  } else {
    guide_yaw = std::atan2(dy, dx);
  }

  const double half_cone = guide_cone_deg_ * M_PI / 360.0;
  const size_t before = frontiers.size();

  if (guide_mode_ == "filter") {
    std::vector<frontier_exploration::Frontier> kept;
    kept.reserve(frontiers.size());
    for (const auto& frontier : frontiers) {
      const double yaw =
          std::atan2(frontier.centroid.y - robot_pose.position.y,
                     frontier.centroid.x - robot_pose.position.x);
      const double diff =
          std::abs(std::remainder(yaw - guide_yaw, 2.0 * M_PI));
      if (diff <= half_cone) {
        kept.push_back(frontier);
      }
    }
    if (!kept.empty()) {
      frontiers.swap(kept);
    } else {
      RCLCPP_WARN_THROTTLE(
          logger_, *this->get_clock(), 2000,
          "No frontier inside the guide cone; falling back to all frontiers.");
    }
  } else {
    for (auto& frontier : frontiers) {
      const double yaw =
          std::atan2(frontier.centroid.y - robot_pose.position.y,
                     frontier.centroid.x - robot_pose.position.x);
      const double diff =
          std::abs(std::remainder(yaw - guide_yaw, 2.0 * M_PI));
      // Penalise frontiers away from the guide direction without excluding
      // them, so exploration still falls back if the cone is empty.
      frontier.cost +=
          guide_weight_ * (1.0 - std::cos(diff)) * frontier.min_distance;
    }
    std::sort(frontiers.begin(), frontiers.end(),
              [](const frontier_exploration::Frontier& a,
                 const frontier_exploration::Frontier& b) {
                return a.cost < b.cost;
              });
  }

  RCLCPP_DEBUG(logger_, "Guide (%s, %.1f deg): %zu -> %zu frontiers.",
               guide_mode_.c_str(), guide_cone_deg_, before, frontiers.size());
}

void Explore::addGuideTarget(
    std::vector<frontier_exploration::Frontier>& frontiers,
    const geometry_msgs::msg::Pose& robot_pose)
{
  if (!guide_navigation_ || !has_guide_) {
    return;
  }
  if (guide_timeout_ > 0.0 &&
      (this->now() - guide_time_).seconds() > guide_timeout_) {
    has_guide_ = false;
    RCLCPP_INFO(logger_, "Direction guide expired.");
    return;
  }

  const double distance = std::hypot(
      guide_.pose.position.x - robot_pose.position.x,
      guide_.pose.position.y - robot_pose.position.y);
  if (distance < guide_min_distance_) {
    return;
  }

  geometry_msgs::msg::Point snapped;
  if (!costmap_client_.findNearestFreeCell(
          guide_.pose.position, goal_snap_radius_, snapped)) {
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 2000,
        "Guide target (%.2f, %.2f) has no free cell within %.1f m.",
        guide_.pose.position.x, guide_.pose.position.y, goal_snap_radius_);
    return;
  }
  if (has_bounds_ && bounds_.size() >= 3 && !pointInPolygon(snapped)) {
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 2000,
        "Guide target (%.2f, %.2f) is outside the exploration boundary; "
        "click inside it or clear the boundary.",
        snapped.x, snapped.y);
    return;
  }

  // A synthetic high-priority frontier: the user pointed at this location, so
  // it wins the normal cost ranking while still passing bounds/blacklist and
  // the planner's own goal-snapping.
  frontier_exploration::Frontier target;
  target.size = 1;
  target.min_distance = distance;
  target.path_distance = distance;
  target.cost = -1e9;
  target.initial = target.centroid = target.middle = snapped;
  target.points.push_back(snapped);
  frontiers.insert(frontiers.begin(), target);

  RCLCPP_INFO_THROTTLE(
      logger_, *this->get_clock(), 2000,
      "Guide target: (%.2f, %.2f), %.2f m away.",
      snapped.x, snapped.y, distance);
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);
  const auto green = std_msgs::msg::ColorRGBA().set__g(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.centroid;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

bool Explore::navigationPosition(double& x, double& y)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(progress_frame_, robot_base_frame_, tf2::TimePointZero);
    const double age = (this->now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (age < -0.1 || age > 1.0) {return false;}
    x = tf.transform.translation.x; y = tf.transform.translation.y;
    return std::isfinite(x) && std::isfinite(y);
  } catch (const tf2::TransformException&) {return false;}
}

void Explore::pauseForImmobility(const char* reason)
{
  goal_active_ = false;
  ++goal_seq_;  // Ignore late results from the canceled navigation goal.
  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();
  frontier_grace_.reset();
  // These destinations were tried while the robot could not leave its origin;
  // they have not been proven unreachable. Allow them again after explicit resume.
  std::vector<geometry_msgs::msg::Point> unproven{committed_goal_};
  frontier_blacklist_.erase(std::remove_if(frontier_blacklist_.begin(),
      frontier_blacklist_.end(), [&](const auto& entry) {
        if (entry.second >= immobility_episode_since_) {
          unproven.push_back(entry.first);
          return true;
        }
        return false;
      }), frontier_blacklist_.end());
  for (auto& attempt : goal_attempts_) {
    for (const auto& point : unproven) {
      if (std::hypot(attempt.centroid.x - point.x, attempt.centroid.y - point.y) < giveup_radius_) {
        attempt.count = 0;
      }
    }
  }
  has_last_sent_ = false;
  auto msg = explore_lite_msgs::msg::ExploreStatus();
  msg.status = "exploration_blocked";
  status_pub_->publish(msg);
  RCLCPP_ERROR(logger_, "Exploration blocked: %s. Not marking exploration complete. "
               "Check robot/map/localization; resume via /explore/resume.", reason);
}

bool Explore::pauseAfterImmobileFailure()
{
  double x = 0.0, y = 0.0;
  const bool valid = navigationPosition(x, y);
  bool origin_blocked = false;
  const auto pose = costmap_client_.getRobotPose();
  if (pose) {
    unsigned int mx, my;
    auto* map = costmap_client_.getCostmap();
    if (map->worldToMap(pose->position.x, pose->position.y, mx, my)) {
      // 253 is inscribed inflation and is normal in a narrow corridor. Smac
      // rejects a 253 start too, but recovery can leave it; only 254/255
      // pauses exploration immediately.
      origin_blocked = map->getCost(mx, my) >= 254;
    }
  }
  const bool was_unresolved = navigation_failure_guard_.unresolved();
  const bool should_pause = navigation_failure_guard_.failed(valid, x, y, origin_blocked);
  if (!was_unresolved && navigation_failure_guard_.unresolved()) {
    immobility_episode_since_ = this->now();
  }
  if (should_pause) {
    pauseForImmobility(!valid ? "local odometry unavailable after navigation failure" :
        origin_blocked ? "no translation after recovery; global origin is lethal/unknown" :
        "repeated navigation failures without translation after recovery");
    return true;
  }
  return false;
}

void Explore::waitForFrontiers(const char* reason, double grace)
{
  const double now = this->now().seconds();
  if (!frontier_grace_.expired(now, grace)) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                        "%s; waiting for map updates (%.1f/%.1f s).",
                        reason, frontier_grace_.elapsed(now), grace);
    return;
  }
  if (navigation_failure_guard_.unresolved()) {
    pauseForImmobility("frontiers exhausted after a navigation failure without translation");
    return;
  }
  RCLCPP_WARN(logger_, "%s after grace period, stopping.", reason);
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
  status_pub_->publish(status_msg);
  stop(true);
}

void Explore::makePlan()
{
  // Track actual odometry motion even between goals: after an aborted goal the
  // robot may be moved by recovery/teleop before the next target is sent.
  double x = 0.0, y = 0.0;
  if (navigationPosition(x, y)) {navigation_failure_guard_.observe(x, y);}
  // Keep the current NavigateToPose goal while its recovery subtree is active.
  // Explicit stop still cancels the action; a missing guard heartbeat expires.
  if (goal_active_ && recovery_active_ &&
      std::chrono::steady_clock::now() - recovery_seen_ < std::chrono::seconds(1)) {
    recovery_was_held_ = true;
    return;
  }
  if (recovery_was_held_) {
    recovery_was_held_ = false;
    last_progress_ = this->now();
    goal_lost_ = false;
    frontier_grace_.reset();
  }
  // An empty/all-unknown costmap is not a valid "exploration complete"
  // condition: it just means mapping has not produced any known cells yet.
  if (!map_ready_seen_) {
    if (!costmap_client_.isMapReady()) {
      RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                           "Costmap has no known cells yet, waiting before "
                           "frontier search.");
      return;
    }
    map_ready_seen_ = true;
    start_time_ = this->now();
    RCLCPP_INFO(logger_,
                "Costmap is ready; starting exploration grace period (%.1f s).",
                startup_grace_);
  }

  auto pose_opt = costmap_client_.getRobotPose();
  if (!pose_opt.has_value()) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                         "Robot pose is not available yet, retrying.");
    return;
  }
  const auto& pose = *pose_opt;

  unsigned int mx, my;
  if (!costmap_client_.getCostmap()->worldToMap(
          pose.position.x, pose.position.y, mx, my)) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                         "Robot pose (%.2f, %.2f) is outside the costmap, "
                         "retrying.",
                         pose.position.x, pose.position.y);
    return;
  }

  if (has_bounds_ && bounds_.size() >= 3 &&
      !pointInPolygon(pose.position)) {
    RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                         "Robot is outside the exploration boundary; holding.");
    return;
  }

  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  // A guide click is a direct navigation target when guide_navigation is on.
  // Add it before the bounds/cone filters so the user's boundary still wins.
  addGuideTarget(frontiers, pose);

  // Keep only frontiers inside the visual boundary, then apply the optional
  // directional guidance bias/filter.
  applyBoundsFilter(frontiers, pose);
  applyGuide(frontiers, pose);

  if (frontiers.empty()) {
    // The committed frontier may have just been cut off by a flickering
    // obstacle; keep driving instead of declaring exploration complete.
    if (holdLostGoal()) {
      return;
    }
    const double elapsed = (this->now() - start_time_).seconds();
    if (elapsed < startup_grace_) {
      RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 2000,
                           "No frontiers yet (%.1f/%.1f s startup grace), "
                           "retrying.",
                           elapsed, startup_grace_);
      return;
    }
    waitForFrontiers("No frontiers found", no_valid_frontier_grace_);
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Pick the first non-blacklisted frontier whose centroid can be snapped to a
  // nearby known traversable cell. Frontier centroids are in unknown space;
  // with allow_unknown=false they cannot be used directly as planner goals.
  auto frontier = frontiers.end();
  geometry_msgs::msg::Point target_position;
  geometry_msgs::msg::Point frontier_centroid;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // A user guide overrides give-up: the user explicitly pointed this way.
    if (goalOnBlacklist(it->centroid) ||
        (!has_guide_ && goalGivenUp(it->centroid))) {
      continue;
    }
    geometry_msgs::msg::Point snapped;
    if (!costmap_client_.findNearestFreeCell(
            it->centroid, goal_snap_radius_, snapped)) {
      RCLCPP_DEBUG(logger_,
                   "Skip frontier (%.2f, %.2f): no free goal within %.1f m.",
                   it->centroid.x, it->centroid.y, goal_snap_radius_);
      continue;
    }
    frontier = it;
    frontier_centroid = it->centroid;
    target_position = snapped;
    break;
  }
  if (frontier == frontiers.end()) {
    // All frontiers were filtered (blacklisted, given-up, or no snap point).
    // This does NOT mean "no frontiers exist" — they may be temporarily
    // inaccessible. Hold for a grace period before declaring complete.
    if (holdLostGoal()) {
      return;
    }
    // Filtered usually means "all remaining candidates are temporarily
    // blacklisted". Do not declare completion before those entries expire.
    waitForFrontiers(
      "All frontiers filtered (blacklist/giveup/no-snap)",
      std::max(no_valid_frontier_grace_, blacklist_timeout_ + 5.0));
    return;
  }

  // Reset the no-valid-frontier grace when a valid frontier is found.
  frontier_grace_.reset();

  // Goal commitment. The ranking can swap between two far-apart branches as
  // the map grows and the robot moves; re-picking the best every cycle then
  // sends the robot back and forth. While a goal is active, find the frontier
  // it belongs to (nearest centroid within goal_track_radius_, since a growing
  // frontier's centroid drifts) and keep it unless the new best is clearly
  // cheaper. Explicit user actions (guide / boundary) skip this once.
  bool keep_current = false;
  if (goal_commit_ && goal_active_ && !drop_commitment_) {
    auto current = frontiers.end();
    double nearest = goal_track_radius_;
    for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
      if (goalOnBlacklist(it->centroid)) {
        continue;
      }
      const double d = std::hypot(it->centroid.x - committed_goal_.x,
                                  it->centroid.y - committed_goal_.y);
      if (d < nearest) {
        nearest = d;
        current = it;
      }
    }
    if (current == frontiers.end()) {
      // The committed frontier is not in the list. It may be explored, or just
      // cut off for a moment by a flickering obstacle. Keep driving for a
      // grace period instead of jumping to another branch and back.
      if (holdLostGoal()) {
        return;
      }
    } else {
      goal_lost_ = false;
      // max(|cost|, 1) keeps the margin meaningful when the cost is near 0
      // (the width term makes it go negative for very close frontiers).
      const double margin =
          switch_improvement_ * std::max(std::abs(current->cost), 1.0);
      if (current == frontier || current->cost - frontier->cost <= margin) {
        if (current != frontier) {
          RCLCPP_DEBUG(logger_,
                       "Keep goal (%.2f, %.2f) cost %.2f over better "
                       "(%.2f, %.2f) cost %.2f: gain below %.0f%%.",
                       current->centroid.x, current->centroid.y, current->cost,
                       frontier->centroid.x, frontier->centroid.y,
                       frontier->cost, switch_improvement_ * 100.0);
        }
        keep_current = true;
        frontier = current;
        frontier_centroid = current->centroid;
        committed_goal_ = frontier_centroid;  // follow the frontier as it grows
      }
    }
  }
  drop_commitment_ = false;

  // time out if we are not making any progress
  bool same_goal = keep_current ||
      same_point(prev_goal_, frontier_centroid, goal_hold_distance_);

  prev_goal_ = frontier_centroid;
  // Use path_distance (BFS along known traversable cells), not the Euclidean
  // min_distance: a detour around a wall can increase straight-line distance
  // while the robot is still making real progress.
  if (!same_goal || prev_distance_ > frontier->path_distance) {
    // we have different goal or we made some progress
    last_progress_ = this->now();
    prev_distance_ = frontier->path_distance;
  }
  // black list if we've made no progress for a long time
  if (goal_active_ &&
      (this->now() - last_progress_ >
       tf2::durationFromSec(progress_timeout_)) &&
      !resuming_) {
    if (pauseAfterImmobileFailure()) {return;}
    frontier_blacklist_.push_back({frontier_centroid, this->now()});
    RCLCPP_DEBUG(logger_, "Adding current goal to black list");
    // Cancel the stuck goal immediately to avoid the 15s goal_lost_grace delay.
    // Without this, makePlan() would see goal_active_=true with no matching
    // frontier, triggering holdLostGoal() and waiting another 15s.
    move_base_client_->async_cancel_all_goals();
    goal_active_ = false;
    makePlan();
    return;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  // we don't need to do anything if we still pursuing the same goal
  if (same_goal && goal_active_) {
    return;
  }

  // Give-up bookkeeping: count how often a region is (re)sent as a goal. A
  // region that keeps coming back without the robot getting in is dropped for
  // giveup_timeout_ seconds. Skipped under a user guide: the user asked for it.
  if (!has_guide_ && max_goal_attempts_ > 0) {
    const auto now = this->now();
    GoalAttempt* attempt = nullptr;
    for (auto& a : goal_attempts_) {
      if (std::hypot(a.centroid.x - frontier_centroid.x,
                     a.centroid.y - frontier_centroid.y) < giveup_radius_) {
        attempt = &a;
        break;
      }
    }
    if (attempt == nullptr) {
      goal_attempts_.push_back({frontier_centroid, 0, now});
      attempt = &goal_attempts_.back();
    } else if ((now - attempt->last).seconds() > giveup_window_) {
      attempt->count = 0;
    }
    // Only count coming BACK to a region after the previous goal was elsewhere
    // (the flip-flop signature). Consecutive goals inside one region, e.g. a
    // frontier that keeps growing as the robot advances, are normal progress.
    const bool returning =
        !has_last_sent_ ||
        std::hypot(last_sent_.x - frontier_centroid.x,
                   last_sent_.y - frontier_centroid.y) >= giveup_radius_;
    if (returning) {
      attempt->count++;
    }
    attempt->last = now;
    if (attempt->count > max_goal_attempts_) {
      RCLCPP_WARN(logger_,
                  "Giving up on region (%.2f, %.2f): sent %d times within "
                  "%.0f s without getting in. Skipping it for %.0f s.",
                  frontier_centroid.x, frontier_centroid.y, attempt->count - 1,
                  giveup_window_, giveup_timeout_);
      giveup_.push_back({frontier_centroid, now});
      attempt->count = 0;
      // Do not send; the next cycle picks another frontier.
      if (goal_active_) {
        move_base_client_->async_cancel_all_goals();
        goal_active_ = false;
      }
      return;
    }
  }

  RCLCPP_INFO(logger_,
              "Frontier goal: centroid=(%.2f, %.2f) snapped=(%.2f, %.2f).",
              frontier_centroid.x, frontier_centroid.y,
              target_position.x, target_position.y);
  goal_lost_ = false;
  last_sent_ = frontier_centroid;
  has_last_sent_ = true;

  RCLCPP_DEBUG(logger_, "Sending goal to move base nav2");

  // send goal to move_base if we have something new to pursue
  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = target_position;
  goal.pose.pose.orientation.w = 1.;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  goal_active_ = true;
  double motion_x = 0.0, motion_y = 0.0;
  const bool motion_valid = navigationPosition(motion_x, motion_y);
  navigation_failure_guard_.begin(motion_valid, motion_x, motion_y);
  committed_goal_ = frontier_centroid;
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // Tag this goal with a monotonically increasing sequence number. Any result
  // belonging to an older goal (preempted) is ignored, which avoids blacklisting
  // valid frontiers on Humble, where a preempted goal reports ABORTED.
  const uint64_t seq = ++goal_seq_;

  send_goal_options.goal_response_callback =
      [this](const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(logger_, "Goal was REJECTED by the action server");
          goal_active_ = false;
        } else {
          active_goal_id_ = goal_handle->get_goal_id();
          RCLCPP_DEBUG(logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
        }
      };

  send_goal_options.result_callback =
      [this, frontier_centroid, seq](
          const NavigationGoalHandle::WrappedResult& result) {
        if (seq != goal_seq_) {
          RCLCPP_DEBUG(logger_,
                       "Ignoring result from a superseded goal (seq %lu != %lu).",
                       seq, goal_seq_);
          return;
        }
        reachedGoal(result, frontier_centroid);
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}

bool Explore::goalGivenUp(const geometry_msgs::msg::Point& goal)
{
  const auto now = this->now();
  giveup_.erase(std::remove_if(giveup_.begin(), giveup_.end(),
                               [&](const auto& entry) {
                                 return (now - entry.second).seconds() >
                                        giveup_timeout_;
                               }),
                giveup_.end());
  for (const auto& entry : giveup_) {
    if (std::hypot(goal.x - entry.first.x, goal.y - entry.first.y) <
        giveup_radius_) {
      return true;
    }
  }
  return false;
}

bool Explore::holdLostGoal()
{
  // Under a user guide the old goal may simply be outside the guided cone, so
  // there is nothing to hold: follow the guide.
  if (!goal_commit_ || !goal_active_ || drop_commitment_ || has_guide_ ||
      goal_lost_grace_ <= 0.0) {
    return false;
  }
  const auto now = this->now();
  if (!goal_lost_) {
    goal_lost_ = true;
    goal_lost_since_ = now;
    RCLCPP_INFO(logger_,
                "Committed frontier (%.2f, %.2f) left the list; holding the "
                "goal for up to %.0f s.",
                committed_goal_.x, committed_goal_.y, goal_lost_grace_);
  }
  return (now - goal_lost_since_).seconds() < goal_lost_grace_;
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  constexpr static size_t tolerace = 5;
  nav2_costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // Expire old entries so a transient failure does not permanently remove a
  // valid frontier from consideration.
  const auto now = this->now();
  frontier_blacklist_.erase(
      std::remove_if(
          frontier_blacklist_.begin(), frontier_blacklist_.end(),
          [&](const auto& entry) {
            return (now - entry.second).seconds() > blacklist_timeout_;
          }),
      frontier_blacklist_.end());

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& entry : frontier_blacklist_) {
    const auto& frontier_goal = entry.first;
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      navigation_failure_guard_.reset();
      RCLCPP_DEBUG(logger_, "Goal was successful");
      last_progress_ = this->now();
      prev_distance_ = 0;
      if (has_guide_ &&
          std::hypot(frontier_goal.x - guide_.pose.position.x,
                     frontier_goal.y - guide_.pose.position.y) <=
          goal_snap_radius_ + 0.5) {
        has_guide_ = false;
        RCLCPP_INFO(
            logger_, "Guide target reached; resuming frontier exploration.");
      }
      break;
    case rclcpp_action::ResultCode::ABORTED:
      if (pauseAfterImmobileFailure()) {return;}
#ifdef NAV2_RESULT_HAS_ERROR_CODE
      if (result.result && result.result->error_code != 0) {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=%d (%s) — blacklisting frontier",
                     result.result->error_code,
                     result.result->error_msg.c_str());
        frontier_blacklist_.push_back({frontier_goal, this->now()});
      } else {
        RCLCPP_DEBUG(logger_, "Goal aborted with error_code=0 — likely a preemption, not blacklisting");
      }
#else
      // Humble: no error_code field, blacklist unconditionally on abort
      RCLCPP_DEBUG(logger_, "Goal aborted — blacklisting frontier");
      frontier_blacklist_.push_back({frontier_goal, this->now()});
#endif
      // If it was aborted probably because we've found another frontier goal,
      // so just return and don't make plan again
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      break;
  }
  // find new goal immediately regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  // oneshot_ = relative_nh_.createTimer(
  //     ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
  //     true);

  // Because of the 1-thread-executor nature of ros2 I think timer is not
  // needed.
  makePlan();
}

void Explore::start()
{
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  goal_active_ = false;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  navigation_failure_guard_.reset();
  resuming_ = true;
  frontier_grace_.reset();
  start_time_ = this->now();  // restart the startup grace window
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
