#include <gtest/gtest.h>

#include "explore/frontier_grace.h"

TEST(FrontierGrace, EmptyAndFilteredSearchesShareOneDeadline)
{
  explore::FrontierGrace grace;
  EXPECT_FALSE(grace.expired(100.0, 60.0));  // empty search
  EXPECT_FALSE(grace.expired(130.0, 60.0));  // all candidates filtered
  EXPECT_TRUE(grace.expired(160.0, 60.0));   // empty again
}

TEST(FrontierGrace, ValidFrontierOrResumeStartsFreshWindow)
{
  explore::FrontierGrace grace;
  EXPECT_FALSE(grace.expired(100.0, 60.0));
  grace.reset();
  EXPECT_FALSE(grace.expired(155.0, 60.0));
  EXPECT_FALSE(grace.expired(214.0, 60.0));
  EXPECT_TRUE(grace.expired(215.0, 60.0));
}

TEST(FrontierGrace, DisabledGraceAndClockRewind)
{
  explore::FrontierGrace grace;
  EXPECT_TRUE(grace.expired(10.0, 0.0));
  EXPECT_FALSE(grace.expired(1.0, 60.0));
  EXPECT_TRUE(grace.expired(61.0, 60.0));
}
