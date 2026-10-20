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
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <explore/frontier_grace.h>
#include <explore/navigation_failure_guard.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false);
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  bool goalGivenUp(const geometry_msgs::msg::Point& goal);
  bool holdLostGoal();
  void waitForFrontiers(const char* reason, double grace);
  bool navigationPosition(double& x, double& y);
  bool pauseAfterImmobileFailure();
  void pauseForImmobility(const char* reason);
  NavigationFailureGuard navigation_failure_guard_;
  std::string progress_frame_{"odom"};
  rclcpp::Time immobility_episode_since_{0, 0, RCL_ROS_TIME};

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Visual boundary polygon and directional guidance.
  void boundsCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void guideCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void clearGuideCallback(const std_msgs::msg::Empty::SharedPtr msg);
  bool pointInPolygon(const geometry_msgs::msg::Point& point) const;
  void applyBoundsFilter(
      std::vector<frontier_exploration::Frontier>& frontiers,
      const geometry_msgs::msg::Pose& robot_pose);
  void applyGuide(std::vector<frontier_exploration::Frontier>& frontiers,
                  const geometry_msgs::msg::Pose& robot_pose);
  void addGuideTarget(std::vector<frontier_exploration::Frontier>& frontiers,
                      const geometry_msgs::msg::Pose& robot_pose);

  std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>>
      frontier_blacklist_;
  geometry_msgs::msg::Point prev_goal_;
  double prev_distance_;
  // 必须显式指定 RCL_ROS_TIME：默认构造是 RCL_SYSTEM_TIME，和 this->now() 相减
  // 会抛 "can't subtract times with different time sources" 让节点直接崩溃。
  rclcpp::Time last_progress_{0, 0, RCL_ROS_TIME};
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  bool resuming_ = false;
  bool goal_active_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recovery_active_sub_;
  bool recovery_active_{false};
  bool recovery_was_held_{false};
  std::chrono::steady_clock::time_point recovery_seen_;
  rclcpp_action::GoalUUID active_goal_id_;

  // Startup handling: an empty/unknown costmap is not "exploration complete".
  bool map_ready_seen_ = false;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double startup_grace_ = 10.0;

  // Goal management: avoid churn that causes preemption/ABORTED blacklisting.
  double goal_hold_distance_ = 0.25;
  double blacklist_timeout_ = 120.0;
  double goal_snap_radius_ = 1.5;
  uint64_t goal_seq_ = 0;

  // Goal commitment: once a frontier goal is sent, keep pursuing it (tracked by
  // proximity, since a growing frontier's centroid drifts) unless a new best
  // frontier is clearly cheaper. Stops the robot from flip-flopping between
  // two far-apart branches whose ranking swaps as the map grows.
  bool goal_commit_ = true;
  double switch_improvement_ = 0.3;   // required relative cost gain to switch
  double goal_track_radius_ = 1.5;    // match radius for "same frontier" (m)
  double max_gain_size_ = 2.0;        // frontier width cap in the cost (m)
  geometry_msgs::msg::Point committed_goal_;
  bool drop_commitment_ = false;      // set by explicit user actions

  // The committed frontier can vanish from the list for a few seconds when a
  // flickering obstacle briefly cuts the path to it. Keep driving for this long
  // before treating it as gone and re-picking.
  double goal_lost_grace_ = 15.0;
  bool goal_lost_ = false;
  rclcpp::Time goal_lost_since_{0, 0, RCL_ROS_TIME};

  // Empty searches and fully filtered searches share this grace period;
  // temporary occlusion must not finish exploration immediately.
  double no_valid_frontier_grace_ = 30.0;
  FrontierGrace frontier_grace_;

  // Give-up: a region that keeps being re-sent as a goal (more than
  // max_goal_attempts_ times within giveup_window_) without the robot getting
  // in is dropped for giveup_timeout_ seconds so exploration moves elsewhere.
  struct GoalAttempt
  {
    geometry_msgs::msg::Point centroid;
    int count;
    rclcpp::Time last;
  };
  int max_goal_attempts_ = 3;
  double giveup_window_ = 180.0;
  double giveup_timeout_ = 600.0;
  double giveup_radius_ = 1.0;
  std::vector<GoalAttempt> goal_attempts_;
  geometry_msgs::msg::Point last_sent_;
  bool has_last_sent_ = false;
  std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> giveup_;

  // Visual boundary polygon (empty = unlimited).
  std::vector<geometry_msgs::msg::Point> bounds_;
  bool has_bounds_ = false;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      bounds_sub_;

  // Soft direction guide: prefer frontiers in this direction.
  geometry_msgs::msg::PoseStamped guide_;
  bool has_guide_ = false;
  rclcpp::Time guide_time_{0, 0, RCL_ROS_TIME};
  double guide_timeout_ = 60.0;
  double guide_weight_ = 2.0;
  double guide_cone_deg_ = 90.0;
  // When true, a guide click becomes a direct high-priority navigation target
  // (snapped to a nearby free cell) instead of only a direction filter.
  bool guide_navigation_ = true;
  double guide_min_distance_ = 0.5;
  std::string guide_mode_ = "bias";
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr guide_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr guide_clear_sub_;
};
}  // namespace explore

#endif
