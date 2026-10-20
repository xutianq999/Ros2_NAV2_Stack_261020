#include <gtest/gtest.h>

#include "recovery_progress.hpp"

using me_nav2_bringup::RecoveryProgress;

TEST(RecoveryProgress, BlockedCommandFailsAtTwoSeconds)
{
  RecoveryProgress progress;
  progress.reset(0.0, 10.0);
  EXPECT_FALSE(progress.stalled(0.0, 11.9));
  EXPECT_TRUE(progress.stalled(0.0, 12.0));
}

TEST(RecoveryProgress, MotionExtendsDeadlineAndCompletionStartsFresh)
{
  RecoveryProgress progress;
  progress.reset(0.0, 0.0);
  EXPECT_FALSE(progress.stalled(0.02, 1.5));
  EXPECT_FALSE(progress.stalled(0.03, 3.0));
  EXPECT_TRUE(progress.stalled(0.03, 5.0));
  progress.reset(0.0, 6.0);
  EXPECT_FALSE(progress.stalled(0.0, 7.0));
}

TEST(RecoveryProgress, JitterDoesNotExtendDeadline)
{
  RecoveryProgress progress;
  progress.reset(0.0, 0.0);
  EXPECT_FALSE(progress.stalled(0.004, 1.0));
  EXPECT_FALSE(progress.stalled(0.008, 1.5));
  EXPECT_TRUE(progress.stalled(0.003, 2.0));
}

TEST(RecoveryProgress, AngularProgressAndClockRewind)
{
  RecoveryProgress progress(2.0, 0.02);
  progress.reset(0.0, 100.0);
  EXPECT_FALSE(progress.stalled(0.03, 101.0));
  EXPECT_FALSE(progress.stalled(0.03, 1.0));
  EXPECT_TRUE(progress.stalled(0.03, 3.0));
}

TEST(RecoveryProgress, RejectInvalidConfiguration)
{
  EXPECT_THROW(RecoveryProgress(0.0, 0.01), std::invalid_argument);
  EXPECT_THROW(RecoveryProgress(2.0, -0.01), std::invalid_argument);
}
