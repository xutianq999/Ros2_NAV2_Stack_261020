#include <gtest/gtest.h>
#include <explore/navigation_failure_guard.h>

TEST(NavigationFailure, FirstBadGoalMayBeSkippedButRepeatedImmobilityPauses)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  EXPECT_FALSE(guard.failed(true, 0.001, 0, false));
  EXPECT_TRUE(guard.unresolved());
  guard.begin(true, 0.001, 0);
  EXPECT_TRUE(guard.failed(true, 0.002, 0, false));
}

TEST(NavigationFailure, BlockedOriginPausesAfterFirstFailedRecovery)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  EXPECT_TRUE(guard.failed(true, 0, 0, true));
  EXPECT_TRUE(guard.unresolved());
}

TEST(NavigationFailure, GenuineTranslationDoesNotPoisonOtherFrontiers)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  guard.observe(0.2, 0);
  EXPECT_FALSE(guard.failed(true, 0.2, 0, false));
  EXPECT_FALSE(guard.unresolved());
}

TEST(NavigationFailure, RecoveryThatReturnsToOriginStillCountsAsMovement)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  guard.observe(-0.1, 0);
  guard.observe(0, 0);
  EXPECT_FALSE(guard.failed(true, 0, 0, false));
  EXPECT_FALSE(guard.unresolved());
}

TEST(NavigationFailure, FailedMotionCannotBeReportedCompleteWithoutResolution)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  EXPECT_FALSE(guard.failed(true, 0, 0, false));
  EXPECT_TRUE(guard.unresolved());
  guard.observe(0.06, 0);
  EXPECT_FALSE(guard.unresolved());
}

TEST(NavigationFailure, UnknownLocalMotionPausesInsteadOfBlacklisting)
{
  explore::NavigationFailureGuard guard;
  guard.begin(false, 0, 0);
  EXPECT_TRUE(guard.failed(false, 0, 0, false));
}

TEST(NavigationFailure, ExplicitResumeStartsFresh)
{
  explore::NavigationFailureGuard guard;
  guard.begin(true, 0, 0);
  EXPECT_TRUE(guard.failed(true, 0, 0, true));
  guard.reset();
  EXPECT_FALSE(guard.unresolved());
  guard.begin(true, 0, 0);
  EXPECT_FALSE(guard.failed(true, 0, 0, false));
}
