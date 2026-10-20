#include <gtest/gtest.h>
#include "escape_trajectory.hpp"
#include "recovery_clearance.hpp"

namespace ex = me_nav2_bringup::escape;

TEST(EscapeTrajectory, OpenSpacePrefersStraightReverse)
{
  auto m = ex::choose({}, {}, [](ex::Pose) {return 0.0;}, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_LT(m.v, 0.0);
  EXPECT_DOUBLE_EQ(m.w, 0.0);
  EXPECT_NEAR(m.poses.back().x, -0.4, 1e-9);
  EXPECT_DOUBLE_EQ(m.poses.back().y, 0.0);
}

// Footprint support along x for a 0.69 x 0.57 m rectangular robot.
double halfX(ex::Pose p)
{
  return 0.345 * std::fabs(std::cos(p.yaw)) + 0.285 * std::fabs(std::sin(p.yaw));
}

TEST(EscapeTrajectory, ShortReverseFitsBeforeRearWall)
{
  auto cost = [](ex::Pose p) {return p.x - halfX(p) <= -0.53 ? 254.0 : 0.0;};
  auto m = ex::choose({}, {}, cost, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_LT(m.v, 0.0);
  EXPECT_NEAR(std::fabs(m.v) * m.duration, 0.15, 1e-9);
  EXPECT_TRUE(ex::safe(m.poses, cost));
}

TEST(EscapeTrajectory, RearBlockedAllowsCheckedForward)
{
  auto cost = [](ex::Pose p) {return p.x - halfX(p) <= -0.36 ? 254.0 : 0.0;};
  auto m = ex::choose({}, {}, cost, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);
  EXPECT_TRUE(ex::safe(m.poses, cost));
}

TEST(EscapeTrajectory, InvalidCostNeverAllowsMotion)
{
  for (double value : {-1.0, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(ex::choose({}, {}, [value](ex::Pose) {return value;}, {}, {}).valid());
  }
}

TEST(EscapeTrajectory, NonPositiveLengthNeverMoves)
{
  ex::Options opt;
  opt.max_length = 0.0;
  EXPECT_FALSE(ex::choose({}, opt, [](ex::Pose) {return 0.0;}, {}, {}).valid());
  opt.max_length = -0.1;
  EXPECT_FALSE(ex::choose({}, opt, [](ex::Pose) {return 0.0;}, {}, {}).valid());
}

TEST(EscapeTrajectory, LethalStartCanEscapeAtEqualCost)
{
  // Being already inside lethal cost must not make every primitive invalid.
  auto m = ex::choose({}, {}, [](ex::Pose) {return 254.0;}, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_LT(m.v, 0.0);
}

TEST(EscapeTrajectory, LethalStartNeverMovesTowardWorseCost)
{
  // Reverse is worse; forward keeps the same lethal score.
  auto m = ex::choose(
    {}, {}, [](ex::Pose p) {return p.x < -0.01 ? 255.0 : 254.0;}, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);
}

TEST(EscapeTrajectory, UnknownStartNeverEntersLethalCell)
{
  // Reverse enters a definite lethal cell (254); forward stays in unknown.
  // 255 must not be treated as "worse than 254" and thereby grant passage.
  auto m = ex::choose(
    {}, {}, [](ex::Pose p) {return p.x < -0.01 ? 254.0 : 255.0;}, {}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);

  const auto entering_lethal = [](ex::Pose p) {return p.x < -0.01 ? 254.0 : 255.0;};
  EXPECT_FALSE(ex::safe({{0, 0, 0}, {-0.1, 0, 0}}, entering_lethal, true));
  EXPECT_TRUE(ex::safe({{0, 0, 0}, {-0.1, 0, 0}},
    [](ex::Pose) {return 255.0;}, true));
}

TEST(EscapeTrajectory, ExecutionCheckAllowsBlockedStartButNotWorse)
{
  const auto constant = [](ex::Pose) {return 254.0;};
  EXPECT_TRUE(ex::safe({{0, 0, 0}, {-0.1, 0, 0}}, constant, true));
  EXPECT_FALSE(ex::safe({{0, 0, 0}, {-0.1, 0, 0}}, constant, false));
  const auto worse_reverse = [](ex::Pose p) {return p.x < -0.01 ? 255.0 : 254.0;};
  EXPECT_FALSE(ex::safe({{0, 0, 0}, {-0.1, 0, 0}}, worse_reverse, true));
}

TEST(EscapeTrajectory, RotationChecksIntermediateFootprint)
{
  auto cost = [](ex::Pose p) {
      if (std::hypot(p.x, p.y) > 0.005) {return 254.0;}
      return std::fabs(p.yaw) > 0.05 && std::fabs(p.yaw) < 0.2 ? 254.0 : 0.0;
    };
  EXPECT_FALSE(ex::choose({}, {}, cost, {}, {}).valid());
}

TEST(EscapeTrajectory, HistoryDoesNotOverrideCurrentObstacle)
{
  auto cost = [](ex::Pose p) {return p.x - halfX(p) <= -0.36 ? 254.0 : 0.0;};
  auto m = ex::choose({}, {}, cost, {{-0.1, 0, 0}, {-0.3, 0, 0}}, {});
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);
}

TEST(EscapeTrajectory, FailedStraightSelectsBoundedArcWithoutSidewaysJump)
{
  ex::Options opt;
  const auto free = [](ex::Pose) {return 0.0;};
  auto straight = ex::choose({}, opt, free, {}, {});
  auto arc = ex::choose({}, opt, free, {}, {straight});
  ASSERT_TRUE(arc.valid());
  EXPECT_LT(arc.v, 0.0);
  EXPECT_NE(arc.w, 0.0);
  EXPECT_LE(std::fabs(arc.w / arc.v), opt.max_curvature);
  EXPECT_LE(std::fabs(arc.w) * arc.duration, opt.max_yaw);
  EXPECT_NEAR(arc.poses.front().yaw, 0.0, 1e-9);
  EXPECT_LT(std::fabs(arc.poses[1].y), std::fabs(arc.poses[1].x) * 0.02);
}

TEST(EscapeTrajectory, SamplingBoundsFootprintCornerDisplacement)
{
  ex::Options opt;
  auto poses = ex::rollout({}, -0.1, 0.15, 4, opt);
  for (size_t i = 1; i < poses.size(); ++i) {
    const auto & a = poses[i - 1]; const auto & b = poses[i];
    double bound = std::hypot(b.x - a.x, b.y - a.y) +
      opt.footprint_radius * std::fabs(ex::wrap(b.yaw - a.yaw));
    EXPECT_LE(bound, opt.sample_step + 1e-9);
  }
}

TEST(EscapeTrajectory, ReverseTrackingMeasuresAlongPathAndLateralErrorSeparately)
{
  auto m = ex::choose({}, {}, [](ex::Pose) {return 0.0;}, {}, {});
  auto t = ex::tracking(m, {-0.1, 0.05, 0.1});
  EXPECT_NEAR(t.progress, 0.1, 1e-9);
  EXPECT_NEAR(t.cross_track, 0.05, 1e-9);
  EXPECT_NEAR(t.yaw_error, -0.1, 1e-9);
}

TEST(EscapeTrajectory, GuidedMotionFollowsStraightPath)
{
  ex::Options opt;
  std::vector<ex::Pose> path{{0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, 0}};
  auto m = ex::guidedMotion({}, path, 0.25, 0.1, opt);
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);
  EXPECT_NEAR(m.w, 0.0, 1e-9);
}

TEST(EscapeTrajectory, GuidedMotionTurnsTowardThePath)
{
  ex::Options opt;
  std::vector<ex::Pose> path{{0, 0, 0}, {0.1, 0.02, 0}, {0.2, 0.04, 0}, {0.3, 0.06, 0}};
  auto m = ex::guidedMotion({}, path, 0.25, 0.1, opt);
  ASSERT_TRUE(m.valid());
  EXPECT_GT(m.v, 0.0);
  EXPECT_GT(m.w, 0.0);
  EXPECT_LE(std::fabs(m.w / m.v), opt.max_curvature + 1e-9);
  EXPECT_LE(std::fabs(m.w) * m.duration, opt.max_yaw + 1e-9);
}

TEST(EscapeTrajectory, GuidedMotionReversesTowardPathBehind)
{
  ex::Options opt;
  std::vector<ex::Pose> path{{0, 0, 0}, {-0.1, 0, 0}, {-0.2, 0, 0}};
  auto m = ex::guidedMotion({}, path, 0.2, 0.1, opt);
  ASSERT_TRUE(m.valid());
  EXPECT_LT(m.v, 0.0);
  EXPECT_NEAR(m.w, 0.0, 1e-9);
}

TEST(EscapeTrajectory, GuidedMotionRejectsExcessiveCurvature)
{
  ex::Options opt;
  std::vector<ex::Pose> path{{0, 0, 0}, {0.1, 0.08, 0}, {0.2, 0.16, 0}};
  EXPECT_FALSE(ex::guidedMotion({}, path, 0.2, 0.1, opt).valid());
}

TEST(EscapeTrajectory, GuidedRotationTurnsTowardPathBehind)
{
  ex::Options opt;
  std::vector<ex::Pose> path{{0, 0, 0}, {-0.1, 0, 0}, {-0.2, 0, 0}};
  auto m = ex::guidedRotation({}, path, 0.3, opt);
  ASSERT_TRUE(m.valid());
  EXPECT_TRUE(m.rotating());
  EXPECT_NE(m.w, 0.0);
  EXPECT_LE(std::fabs(m.w) * m.duration, 0.26 + 1e-9);
}

TEST(RecoveryClearance, ShortFiveCentimeterReverseFitsGuardClearance)
{
  ex::Options opt;
  std::vector<ex::ScanPoint> scan{{-0.48, -0.05}, {-0.48, 0.05}};
  auto m = ex::choose({}, opt, [](ex::Pose) {return 0.0;}, {}, {}, nullptr,
      [&](const std::vector<ex::Pose> & poses) {return ex::scanSafe(poses, scan, {});});
  ASSERT_TRUE(m.valid());
  EXPECT_LT(m.v, 0.0);
  EXPECT_NEAR(std::fabs(m.v) * m.duration, 0.05, 1e-9);
  EXPECT_TRUE(ex::scanSafe(m.poses, scan, {}));
}

TEST(RecoveryClearance, TightSideWallAllowsTangentialEscape)
{
  ex::Options opt;
  std::vector<ex::ScanPoint> scan{{-0.2, 0.295}, {0.2, 0.295}};
  EXPECT_TRUE(ex::scanSafe(ex::rollout({}, -0.1, 0, 1, opt), scan, {}));
  EXPECT_TRUE(ex::scanSafe(ex::rollout({}, 0.1, 0, 1, opt), scan, {}));
}

TEST(RecoveryClearance, CloseCornerCanRotateAwayButNeverCloser)
{
  ex::Options opt;
  std::vector<ex::ScanPoint> scan{{0.345, 0.29}, {0.34, 0.292}, {0.335, 0.295}};
  EXPECT_TRUE(ex::scanSafe(ex::rollout({}, 0, -0.5, 0.5, opt), scan, {}));
  EXPECT_FALSE(ex::scanSafe(ex::rollout({}, 0, 0.5, 0.5, opt), scan, {}));
}

TEST(RecoveryClearance, OverlapNeverGetsEscapeExemption)
{
  ex::Options opt;
  std::vector<ex::ScanPoint> scan{{0.05, 0.05}, {0.08, 0.02}};
  for (double v : {-0.1, 0.1}) {
    EXPECT_FALSE(ex::scanSafe(ex::rollout({}, v, 0, 0.5, opt), scan, {}));
  }
}

TEST(RecoveryClearance, WorldCoordinatesAndRotationKeepTheSameDecision)
{
  const ex::Pose origin{1.0, -0.4, 0.7};
  std::vector<ex::ScanPoint> scan;
  for (double y : {-0.05, 0.05}) {
    scan.push_back({origin.x - 0.48 * std::cos(origin.yaw) - y * std::sin(origin.yaw),
      origin.y - 0.48 * std::sin(origin.yaw) + y * std::cos(origin.yaw)});
  }
  EXPECT_TRUE(ex::scanSafe(ex::rollout(origin, -0.1, 0, 0.5, {}), scan, {}));
  EXPECT_FALSE(ex::scanSafe(ex::rollout(origin, -0.1, 0, 1.5, {}), scan, {}));
}

TEST(RecoveryClearance, ShortLookaheadCanCreepWithoutChangingCurvature)
{
  std::vector<ex::ScanPoint> scan{{-0.42, -0.01}, {-0.42, 0.01}};
  auto cmd = ex::limitByScan({}, -0.1, 0.1, scan, {}, {});
  ASSERT_TRUE(cmd.valid());
  EXPECT_GT(cmd.v, -0.1);
  EXPECT_LE(cmd.v, -0.02);
  EXPECT_NEAR(cmd.w / cmd.v, -1.0, 1e-9);
  EXPECT_TRUE(ex::scanSafe(cmd.poses, scan, {}));
}

TEST(RecoveryClearance, CostmapCanVetoEveryCreepingSpeed)
{
  std::vector<ex::ScanPoint> scan{{2, 2}};
  auto cmd = ex::limitByScan({}, -0.1, 0.1, scan, {}, {},
      [](const std::vector<ex::Pose> &) {return false;});
  EXPECT_FALSE(cmd.valid());
}
