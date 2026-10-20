#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "escape_grid.hpp"

namespace ex = me_nav2_bringup::escape;

namespace
{
struct TestGrid
{
  TestGrid(unsigned int w, unsigned int h, unsigned char value = 0)
  : data(static_cast<size_t>(w) * h, value), width(w), height(h) {}

  ex::GridView view(double resolution = 0.1)
  {
    return {data.data(), width, height, resolution, 0.0, 0.0};
  }

  unsigned char & at(int x, int y)
  {
    return data[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
  }

  std::vector<unsigned char> data;
  unsigned int width, height;
};

void fillRect(TestGrid & grid, int x0, int y0, int x1, int y1, unsigned char value)
{
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      grid.at(x, y) = value;
    }
  }
}
}  // namespace

TEST(EscapeGrid, AlreadyTraversableReturnsStart)
{
  TestGrid grid(5, 5, 0);
  const auto path = ex::nearestTraversablePath(grid.view(), 2, 2, {});
  ASSERT_TRUE(path.found);
  ASSERT_EQ(path.cells.size(), 1u);
  EXPECT_EQ(path.cells.front().x, 2);
  EXPECT_EQ(path.cells.front().y, 2);
  EXPECT_FALSE(path.crossed_blocked);
}

TEST(EscapeGrid, BlockedStartUsesRelaxedPass)
{
  TestGrid grid(9, 9, 0);
  fillRect(grid, 3, 3, 5, 5, 254);
  ex::GridPathOptions opt;
  const auto path = ex::nearestTraversablePath(grid.view(), 4, 4, opt);
  ASSERT_TRUE(path.found);
  ASSERT_GE(path.cells.size(), 2u);
  EXPECT_EQ(path.cells.front().x, 4);
  EXPECT_EQ(path.cells.front().y, 4);
  EXPECT_LE(grid.at(path.cells.back().x, path.cells.back().y), 252);
  EXPECT_TRUE(path.crossed_blocked);
  EXPECT_GT(path.length, 0.0);
}

TEST(EscapeGrid, RelaxedPassCanBeDisabled)
{
  TestGrid grid(9, 9, 0);
  fillRect(grid, 3, 3, 5, 5, 254);
  ex::GridPathOptions opt;
  opt.allow_blocked = false;
  EXPECT_FALSE(ex::nearestTraversablePath(grid.view(), 4, 4, opt).found);
}

TEST(EscapeGrid, UnknownIsNotCrossedUnlessAllowed)
{
  TestGrid grid(11, 11, 0);
  // 254 core, 255 ring, 0 border: leaving the core requires crossing unknown.
  fillRect(grid, 2, 2, 8, 8, 255);
  fillRect(grid, 3, 3, 7, 7, 254);

  ex::GridPathOptions opt;
  opt.allow_blocked = true;
  opt.allow_unknown = false;
  EXPECT_FALSE(ex::nearestTraversablePath(grid.view(), 5, 5, opt).found);

  opt.allow_unknown = true;
  const auto path = ex::nearestTraversablePath(grid.view(), 5, 5, opt);
  ASSERT_TRUE(path.found);
  EXPECT_LE(grid.at(path.cells.back().x, path.cells.back().y), 252);
  EXPECT_TRUE(path.crossed_blocked);
}

TEST(EscapeGrid, WallIsCrossedOnlyThroughTheGap)
{
  TestGrid grid(11, 11, 254);
  // Free region on the right, one free gap in the wall at x = 5.
  fillRect(grid, 6, 0, 10, 10, 0);
  grid.at(5, 5) = 0;
  ex::GridPathOptions opt;
  opt.allow_blocked = false;
  const auto path = ex::nearestTraversablePath(grid.view(), 4, 5, opt);
  ASSERT_TRUE(path.found);
  bool used_gap = false;
  for (size_t i = 1; i < path.cells.size(); ++i) {
    const auto & cell = path.cells[i];
    EXPECT_LT(grid.at(cell.x, cell.y), 253);
    if (cell.x == 5 && cell.y == 5) {used_gap = true;}
  }
  EXPECT_TRUE(used_gap);
}

TEST(EscapeGrid, SearchRadiusLimitsTheResult)
{
  TestGrid grid(11, 11, 254);
  for (int i = 0; i < 11; ++i) {
    grid.at(i, 0) = grid.at(i, 10) = 0;
    grid.at(0, i) = grid.at(10, i) = 0;
  }
  ex::GridPathOptions opt;
  opt.allow_blocked = true;
  opt.max_search_distance = 0.2;
  EXPECT_FALSE(ex::nearestTraversablePath(grid.view(), 5, 5, opt).found);

  opt.max_search_distance = 2.0;
  EXPECT_TRUE(ex::nearestTraversablePath(grid.view(), 5, 5, opt).found);
}

TEST(EscapeGrid, MaxCellsLimitsTheResult)
{
  TestGrid grid(11, 11, 254);
  for (int i = 0; i < 11; ++i) {
    grid.at(i, 0) = grid.at(i, 10) = 0;
    grid.at(0, i) = grid.at(10, i) = 0;
  }
  ex::GridPathOptions opt;
  opt.max_cells = 3;
  EXPECT_FALSE(ex::nearestTraversablePath(grid.view(), 5, 5, opt).found);
}

TEST(EscapeGrid, ReconstructionRunsFromStartToTarget)
{
  TestGrid grid(9, 9, 0);
  fillRect(grid, 3, 3, 5, 5, 254);
  ex::GridPathOptions opt;
  const auto path = ex::nearestTraversablePath(grid.view(), 4, 4, opt);
  ASSERT_TRUE(path.found);
  EXPECT_EQ(path.cells.front().x, 4);
  EXPECT_EQ(path.cells.front().y, 4);

  double length = 0.0;
  for (size_t i = 1; i < path.cells.size(); ++i) {
    length += std::hypot(
      path.cells[i].x - path.cells[i - 1].x,
      path.cells[i].y - path.cells[i - 1].y) * 0.1;
  }
  EXPECT_NEAR(path.length, length, 1e-9);
  EXPECT_LE(grid.at(path.cells.back().x, path.cells.back().y), 252);
}
