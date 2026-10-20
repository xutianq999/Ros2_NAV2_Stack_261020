#ifndef ME_NAV2_BRINGUP__RECOVERY_PROGRESS_HPP_
#define ME_NAV2_BRINGUP__RECOVERY_PROGRESS_HPP_

#include <cmath>
#include <stdexcept>

namespace me_nav2_bringup
{
// Watch actual cumulative progress, so repeated zero commands and pose jitter
// cannot keep an obstructed recovery alive until its full action timeout.
class RecoveryProgress
{
public:
  RecoveryProgress(double timeout = 2.0, double epsilon = 0.01)
  : timeout_(timeout), epsilon_(epsilon)
  {
    if (!std::isfinite(timeout) || timeout <= 0.0 ||
      !std::isfinite(epsilon) || epsilon <= 0.0)
    {
      throw std::invalid_argument("Recovery progress timeout/epsilon must be finite and positive");
    }
  }

  void reset(double metric, double now)
  {
    best_ = metric;
    last_progress_ = now;
  }

  bool stalled(double metric, double now)
  {
    if (!std::isfinite(metric) || !std::isfinite(now)) {
      return true;
    }
    if (now < last_progress_ || metric >= best_ + epsilon_) {
      reset(metric, now);
    }
    return now - last_progress_ >= timeout_;
  }

private:
  double timeout_;
  double epsilon_;
  double best_{0.0};
  double last_progress_{0.0};
};
}  // namespace me_nav2_bringup

#endif  // ME_NAV2_BRINGUP__RECOVERY_PROGRESS_HPP_
