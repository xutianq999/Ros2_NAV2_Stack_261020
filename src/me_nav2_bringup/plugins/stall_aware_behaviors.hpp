#ifndef ME_NAV2_BRINGUP__STALL_AWARE_BEHAVIORS_HPP_
#define ME_NAV2_BRINGUP__STALL_AWARE_BEHAVIORS_HPP_

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_behaviors/plugins/back_up.hpp"
#include "nav2_behaviors/plugins/spin.hpp"
#include "nav2_util/node_utils.hpp"
#include "recovery_progress.hpp"

namespace me_nav2_bringup
{
// Retain Nav2's action interfaces, feedback, collision checks and cancellation.
// Add an actual-motion deadline, including when recovery_guard or a mux lock
// prevents the command from reaching the chassis.
template<typename Base, typename Action>
class StallAwareBehavior : public Base
{
public:
  void onConfigure() override
  {
    Base::onConfigure();
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error("Recovery behavior node is unavailable");
    }
    const std::string prefix = this->behavior_name_ + ".";
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + "stall_timeout", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(
      node, prefix + "progress_epsilon", rclcpp::ParameterValue(defaultEpsilon()));
    progress_ = RecoveryProgress(
      node->get_parameter(prefix + "stall_timeout").as_double(),
      node->get_parameter(prefix + "progress_epsilon").as_double());
    // Both the local collision checker and motion measurements use odom.
    nav2_util::declare_parameter_if_not_declared(
      node, "local_frame", rclcpp::ParameterValue(std::string("odom")));
    this->global_frame_ = node->get_parameter("local_frame").as_string();
  }

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const typename Action::Goal> command) override
  {
    const auto status = Base::onRun(command);
    if (status == nav2_behaviors::Status::SUCCEEDED) {
      progress_.reset(0.0, this->clock_->now().seconds());
    }
    return status;
  }

  nav2_behaviors::Status onCycleUpdate() override
  {
    const auto status = Base::onCycleUpdate();
    if (status != nav2_behaviors::Status::RUNNING) {
      // Also stop on TF errors: upstream versions do not all send a zero here.
      this->stopRobot();
      return status;
    }
    if (progress_.stalled(progressMetric(), this->clock_->now().seconds())) {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_, "%s: no actual progress; trying the next recovery",
        this->behavior_name_.c_str());
      return nav2_behaviors::Status::FAILED;
    }
    return status;
  }

protected:
  virtual double progressMetric() const = 0;
  virtual double defaultEpsilon() const = 0;
  RecoveryProgress progress_;
};

class StallAwareBackUp
  : public StallAwareBehavior<nav2_behaviors::BackUp, nav2_msgs::action::BackUp>
{
protected:
  double progressMetric() const override {return feedback_->distance_traveled;}
  double defaultEpsilon() const override {return 0.01;}
};

class StallAwareSpin
  : public StallAwareBehavior<nav2_behaviors::Spin, nav2_msgs::action::Spin>
{
protected:
  double progressMetric() const override {return std::fabs(relative_yaw_);}
  double defaultEpsilon() const override {return 0.02;}
};
}  // namespace me_nav2_bringup

#endif  // ME_NAV2_BRINGUP__STALL_AWARE_BEHAVIORS_HPP_
