#ifndef EXPLORE_FRONTIER_GRACE_H_
#define EXPLORE_FRONTIER_GRACE_H_

#include <algorithm>

namespace explore
{
// Shared by empty frontier searches and searches whose candidates are all
// filtered. Switching between those conditions must not restart the deadline.
class FrontierGrace
{
public:
  void reset() {waiting_ = false;}

  bool expired(double now, double grace)
  {
    if (!waiting_ || now < since_) {
      waiting_ = true;
      since_ = now;
    }
    return elapsed(now) >= std::max(0.0, grace);
  }

  double elapsed(double now) const {return std::max(0.0, now - since_);}

private:
  bool waiting_{false};
  double since_{0.0};
};
}  // namespace explore

#endif  // EXPLORE_FRONTIER_GRACE_H_
