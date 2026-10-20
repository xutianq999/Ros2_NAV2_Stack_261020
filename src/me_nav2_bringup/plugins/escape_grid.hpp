#ifndef ME_NAV2_BRINGUP__ESCAPE_GRID_HPP_
#define ME_NAV2_BRINGUP__ESCAPE_GRID_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace me_nav2_bringup
{
namespace escape
{
// Row-major costmap snapshot. Costs follow Nav2 conventions:
//   0..252 traversable, 253 inscribed, 254 lethal, 255 unknown.
struct GridView
{
  const unsigned char * data{nullptr};
  unsigned int width{0}, height{0};
  double resolution{0.05}, origin_x{0.0}, origin_y{0.0};

  bool valid() const
  {
    return data != nullptr && width > 0 && height > 0 &&
           std::isfinite(resolution) && resolution > 0.0;
  }

  bool inside(int x, int y) const
  {
    return x >= 0 && y >= 0 &&
           static_cast<unsigned int>(x) < width &&
           static_cast<unsigned int>(y) < height;
  }

  unsigned char at(int x, int y) const
  {
    return data[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
  }

  bool worldToMap(double wx, double wy, int & mx, int & my) const
  {
    const double x = (wx - origin_x) / resolution;
    const double y = (wy - origin_y) / resolution;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0) {return false;}
    mx = static_cast<int>(std::floor(x));
    my = static_cast<int>(std::floor(y));
    return inside(mx, my);
  }

  double cellX(int x) const {return origin_x + (x + 0.5) * resolution;}
  double cellY(int y) const {return origin_y + (y + 0.5) * resolution;}
};

struct GridCell
{
  int x{0}, y{0};
};

struct GridPathOptions
{
  // Target is the normal traversable region: cost <= target_cost. With the
  // default 252 this means "anything below INSCRIBED(253)".
  double target_cost{252.0};
  // Extra cost for crossing 253/254. High enough to prefer a detour, low
  // enough that a fully enclosed robot still gets a path.
  double blocked_penalty{20.0};
  double unknown_penalty{10.0};
  bool allow_blocked{true};
  bool allow_unknown{false};
  double max_search_distance{5.0};
  size_t max_cells{50000};
};

struct GridPath
{
  bool found{false};
  bool crossed_blocked{false};
  std::vector<GridCell> cells;  // start -> target
  double length{0.0};
  double cost{0.0};
};

namespace grid_detail
{
struct Node
{
  int index;
  double cost;
  bool operator>(const Node & other) const {return cost > other.cost;}
};

inline double entryPenalty(unsigned char cost, const GridPathOptions & opt)
{
  if (cost >= 253) {
    return cost == 255 ? opt.unknown_penalty : opt.blocked_penalty;
  }
  // Mild preference for lower inflation cost.
  return static_cast<double>(cost) / 252.0 * 2.0;
}

inline bool passable(
  unsigned char cost, const GridPathOptions & opt, bool relaxed)
{
  if (!relaxed) {return cost < 253;}
  if (cost == 255) {return opt.allow_unknown;}
  if (cost >= 253) {return opt.allow_blocked;}
  return true;
}
}  // namespace grid_detail

// Dijkstra over the costmap. The start cell is always seeded, so a robot that
// is already inside 253/254/255 can still search outward. Pass 1 only crosses
// normal traversable cells; if that fails and blocking is allowed, pass 2
// permits 253/254 (and optionally 255) at a high penalty. The returned path
// is ordered start -> nearest reachable cell with cost <= target_cost.
inline GridPath nearestTraversablePath(
  const GridView & grid, int start_x, int start_y, const GridPathOptions & opt)
{
  GridPath result;
  if (!grid.valid() || !grid.inside(start_x, start_y) ||
    !std::isfinite(opt.max_search_distance) || opt.max_search_distance <= 0.0 ||
    !std::isfinite(opt.target_cost) || opt.target_cost < 0.0 ||
    opt.max_cells == 0)
  {
    return result;
  }

  if (grid.at(start_x, start_y) <= opt.target_cost) {
    result.found = true;
    result.cells.push_back({start_x, start_y});
    return result;
  }

  const size_t total = static_cast<size_t>(grid.width) * grid.height;
  const int width = static_cast<int>(grid.width);
  const int start_index = start_y * width + start_x;
  const double start_wx = grid.cellX(start_x);
  const double start_wy = grid.cellY(start_y);
  const double max_distance = opt.max_search_distance;

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int pass = 0; pass < 2; ++pass) {
    const bool relaxed = pass == 1;
    if (relaxed && !opt.allow_blocked && !opt.allow_unknown) {break;}

    std::vector<double> best(total, std::numeric_limits<double>::infinity());
    std::vector<int> parent(total, -1);
    std::vector<unsigned char> closed(total, 0);
    std::priority_queue<
      grid_detail::Node, std::vector<grid_detail::Node>,
      std::greater<grid_detail::Node>> queue;

    best[start_index] = 0.0;
    queue.push({start_index, 0.0});
    size_t visited = 0;
    int target_index = -1;

    while (!queue.empty() && visited < opt.max_cells) {
      const grid_detail::Node node = queue.top();
      queue.pop();
      if (closed[node.index] || node.cost > best[node.index] + 1e-12) {continue;}
      closed[node.index] = 1;
      ++visited;

      const int x = node.index % width;
      const int y = node.index / width;
      const unsigned char cost = grid.at(x, y);
      const double distance_from_start =
        std::hypot(grid.cellX(x) - start_wx, grid.cellY(y) - start_wy);
      if (distance_from_start > max_distance) {continue;}

      if (node.index != start_index && cost <= opt.target_cost) {
        target_index = node.index;
        break;
      }

      for (int k = 0; k < 8; ++k) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];
        if (!grid.inside(nx, ny)) {continue;}
        const unsigned char nc = grid.at(nx, ny);
        if (!grid_detail::passable(nc, opt, relaxed)) {continue;}

        const bool diagonal = dx[k] != 0 && dy[k] != 0;
        if (diagonal) {
          // Do not cut between two blocked orthogonal cells.
          if (!grid_detail::passable(grid.at(x + dx[k], y), opt, relaxed) ||
            !grid_detail::passable(grid.at(x, y + dy[k]), opt, relaxed))
          {
            continue;
          }
        }

        const double step = diagonal ? std::sqrt(2.0) : 1.0;
        const int next = ny * width + nx;
        const double candidate = node.cost + step *
          (1.0 + grid_detail::entryPenalty(nc, opt));
        if (candidate + 1e-12 < best[next]) {
          best[next] = candidate;
          parent[next] = node.index;
          queue.push({next, candidate});
        }
      }
    }

    if (target_index < 0) {continue;}

    bool crossed = false;
    for (int index = target_index; index >= 0; index = parent[index]) {
      const unsigned char cost = grid.at(index % width, index / width);
      if (index != start_index && cost >= 253) {crossed = true;}
      result.cells.push_back({index % width, index / width});
      if (index == start_index) {break;}
    }
    std::reverse(result.cells.begin(), result.cells.end());
    result.found = !result.cells.empty();
    result.crossed_blocked = crossed;
    result.cost = best[target_index];
    result.length = 0.0;
    for (size_t i = 1; i < result.cells.size(); ++i) {
      const double cdx = result.cells[i].x - result.cells[i - 1].x;
      const double cdy = result.cells[i].y - result.cells[i - 1].y;
      result.length += std::hypot(cdx, cdy) * grid.resolution;
    }
    return result;
  }
  return result;
}
}  // namespace escape
}  // namespace me_nav2_bringup

#endif  // ME_NAV2_BRINGUP__ESCAPE_GRID_HPP_
