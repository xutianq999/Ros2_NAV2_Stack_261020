#ifndef ME_NAV2_BRINGUP__SMART_RECOVERY_HPP_
#define ME_NAV2_BRINGUP__SMART_RECOVERY_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "escape_grid.hpp"
#include "escape_trajectory.hpp"
#include "recovery_clearance.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace me_nav2_bringup
{
// Short, nonholonomic escape primitives with closed-loop tracking. Retains the
// BackUp action interface used by the BT, including cancellation and time limit.
class SmartRecovery : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion() override;
  void activate() override;
  void deactivate() override;
  nav2_behaviors::Status onRun(
    const std::shared_ptr<const nav2_msgs::action::BackUp::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

private:
  bool getPose(escape::Pose & pose);
  void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  bool scanSnapshot(std::vector<escape::ScanPoint> & points);
  double poseCost(const escape::Pose & pose, bool fetch);
  bool planFrom(const escape::Pose & pose);
  bool planPrimitives(const escape::Pose & pose);
  bool planFromFreeSpace(const escape::Pose & pose);
  bool selectGuidedMotion(const escape::Pose & pose);
  bool validateFreePath(
    const escape::Pose & pose, std::vector<escape::Pose> & path,
    const std::vector<escape::ScanPoint> & scan);
  bool globalCost(const escape::Pose & pose, double & cost);
  void onFreeCostmap(nav2_msgs::msg::Costmap::SharedPtr msg);
  bool costmapTransform(
    const std::string & frame, double & tx, double & ty, double & yaw);
  nav2_behaviors::Status retry(const escape::Pose & pose, const char * reason);
  nav2_behaviors::Status finish(nav2_behaviors::Status status);
  void rememberPose();
  void publishPlan(const std::vector<escape::Motion> & candidates);
  void clearPlan();

  bool smart_enabled_{true};
  escape::Options options_;
  escape::Clearance clearance_;
  double scan_timeout_{0.5};
  std::mutex scan_mutex_;
  std::vector<escape::ScanPoint> scan_points_;
  std::chrono::steady_clock::time_point scan_time_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double backup_distance_{0.5}, backup_speed_{0.1};
  double stall_timeout_{2.0}, execute_distance_{0.1};
  double yaw_gain_{1.5}, cross_track_gain_{2.0}, max_cross_track_{0.06};
  double max_yaw_error_{0.26}, max_rotation_{0.78};
  double min_exit_distance_{0.05}, exit_cost_{180.0};
  int max_replans_{4}, replan_count_{0};
  std::string local_frame_{"odom"};

  // Free-space escape: search the global costmap for the nearest normal
  // traversable cell and follow a path to it when the start is blocked.
  bool free_space_escape_{true};
  std::string free_space_costmap_topic_{"/global_costmap/costmap_raw"};
  std::string free_space_costmap_frame_{"map"};
  double free_space_costmap_timeout_{1.0};
  double free_space_trigger_cost_{254.0};
  double free_space_target_cost_{252.0};
  double free_space_search_radius_{5.0};
  double free_space_max_distance_{2.5};
  double free_space_lookahead_{0.35};
  bool free_space_allow_blocked_{true};
  bool free_space_allow_unknown_{false};
  double free_space_blocked_penalty_{20.0};
  size_t free_space_max_cells_{50000};
  double free_space_max_total_rotation_{3.14};
  double max_time_allowance_{40.0};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr free_costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr free_costmap_;
  rclcpp::Time free_costmap_stamp_;
  std::mutex free_costmap_mutex_;
  bool path_mode_{false};
  bool path_crossed_blocked_{false};
  std::vector<escape::Pose> free_path_;

  escape::Motion motion_;
  std::vector<escape::Motion> rejected_;
  escape::Pose action_start_;
  double distance_{0.5}, speed_{0.1}, allowance_{20.0};
  double travelled_{0.0}, rotated_{0.0}, segment_progress_{0.0};
  double progress_mark_{0.0};
  rclcpp::Time start_time_, last_progress_time_;
  std::atomic<bool> running_{false};
  std::mutex history_mutex_;
  std::deque<std::pair<double, escape::Pose>> history_;
  rclcpp::TimerBase::SharedPtr history_timer_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_pub_;
};
}  // namespace me_nav2_bringup

#endif  // ME_NAV2_BRINGUP__SMART_RECOVERY_HPP_
