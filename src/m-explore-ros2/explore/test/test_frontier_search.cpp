// FrontierSearch 的代价应按"沿已知可通行格走过去的路径距离"排序，
// 而不是直线距离：隔着一堵墙的前沿直线很近、路径很远，不该排在前面。
//
// 合成地图（0.05 m/格，全部先填成障碍，再挖出可通行区域）：
//
//   y=80 ┌─ 上走廊 (x15..95) ──────────────┐
//        │  A 开口在 x=14（未知）          │ 右侧竖直连接 (x90..95)
//   y=55 │                                 │
//   y=45 └─ 下走廊 (x5..95) ── B 开口 x=96 ┘
//        机器人在 (10,50)
//
// A 离机器人直线约 1.35 m，但要走完整个回路，路径约 9.5 m；
// B 就在下走廊尽头，直线和路径都约 4.3 m。按路径距离，B 应当排第一。

#include <gtest/gtest.h>

#include <explore/frontier_search.h>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>

namespace
{
constexpr double kRes = 0.05;
constexpr unsigned int kSize = 100;

void fill(nav2_costmap_2d::Costmap2D& map, unsigned int x0, unsigned int x1,
          unsigned int y0, unsigned int y1, unsigned char value)
{
  for (unsigned int y = y0; y <= y1; ++y) {
    for (unsigned int x = x0; x <= x1; ++x) {
      map.setCost(x, y, value);
    }
  }
}

nav2_costmap_2d::Costmap2D makeLoopMap()
{
  nav2_costmap_2d::Costmap2D map(kSize, kSize, kRes, 0.0, 0.0,
                                 nav2_costmap_2d::LETHAL_OBSTACLE);
  fill(map, 5, 95, 45, 55, nav2_costmap_2d::FREE_SPACE);    // 下走廊
  fill(map, 90, 95, 56, 80, nav2_costmap_2d::FREE_SPACE);   // 右侧竖直连接
  fill(map, 15, 95, 75, 80, nav2_costmap_2d::FREE_SPACE);   // 上走廊
  fill(map, 96, 96, 45, 55, nav2_costmap_2d::NO_INFORMATION);  // B 开口
  fill(map, 14, 14, 75, 80, nav2_costmap_2d::NO_INFORMATION);  // A 开口
  return map;
}

geometry_msgs::msg::Point robotAt(unsigned int cx, unsigned int cy)
{
  geometry_msgs::msg::Point p;
  p.x = (cx + 0.5) * kRes;
  p.y = (cy + 0.5) * kRes;
  return p;
}
}  // namespace

TEST(FrontierSearch, RanksByPathDistanceNotStraightLine)
{
  auto map = makeLoopMap();
  frontier_exploration::FrontierSearch search(
    &map, 3.0, 1.0, 0.2, 2.0, rclcpp::get_logger("test"));

  const auto frontiers = search.searchFrom(robotAt(10, 50));
  ASSERT_EQ(frontiers.size(), 2u);

  // 第一名应是下走廊尽头的 B（x≈4.8 m），而不是隔墙的 A（x≈0.7 m）。
  EXPECT_GT(frontiers[0].centroid.x, 4.0);
  EXPECT_LT(frontiers[1].centroid.x, 1.0);

  // A 的路径距离必须远大于它的直线距离，否则这个测试没测到东西。
  const auto& a = frontiers[1];
  EXPECT_GT(a.path_distance, 2.0 * a.min_distance);
  // B 沿走廊直走，路径距离和直线距离应基本一致。
  const auto& b = frontiers[0];
  EXPECT_NEAR(b.path_distance, b.min_distance, 0.5);
}

TEST(FrontierSearch, WidthGainIsCapped)
{
  // 同一张图，把宽度上限设得很小：收益被压平，代价应由路径距离主导，
  // 排序结果不变，且代价值 = 3·path − 1·min(width, cap)。
  auto map = makeLoopMap();
  frontier_exploration::FrontierSearch search(
    &map, 3.0, 1.0, 0.2, 0.1, rclcpp::get_logger("test"));

  const auto frontiers = search.searchFrom(robotAt(10, 50));
  ASSERT_EQ(frontiers.size(), 2u);
  for (const auto& f : frontiers) {
    EXPECT_NEAR(f.cost, 3.0 * f.path_distance - 0.1, 1e-6);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
