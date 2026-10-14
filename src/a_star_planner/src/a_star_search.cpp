// The A* search itself. See a_star_search.hpp and Lesson 10 for the requirements.

#include "a_star_planner/a_star_search.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace robonav_training
{
namespace
{

bool isBlocked(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int col, int row)
{
  return !grid.inBounds(col, row) || blocked[grid.index(col, row)] != 0;
}

}  // namespace

std::vector<int> astarSearch(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int start_col, int start_row, int goal_col, int goal_row)
{
  // Part 1 is provided: reject malformed inputs and impossible endpoints.
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
    blocked.size() != grid.cellCount() ||
    !grid.inBounds(start_col, start_row) || !grid.inBounds(goal_col, goal_row) ||
    isBlocked(grid, blocked, start_col, start_row) ||
    isBlocked(grid, blocked, goal_col, goal_row))
  {
    return {};
  }

  const std::size_t cell_count = grid.cellCount();
  const std::size_t start_index = grid.index(start_col, start_row);
  const std::size_t goal_index = grid.index(goal_col, goal_row);
  std::vector<double> g_score(
    cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  g_score[start_index] = 0.0;

  auto heuristic = [&](int col, int row) -> double {
    const double dc = static_cast<double>(col - goal_col);
    const double dr = static_cast<double>(row - goal_row);
    return std::sqrt(dc * dc + dr * dr) * grid.resolution;
  };

  struct Entry {
    double f;
    int col;
    int row;
  };

  auto cmp = [](const Entry & a, const Entry & b) { return a.f > b.f; };
  std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> open(cmp);

  open.push({heuristic(start_col, start_row), start_col, start_row});

  while (!open.empty()) {
    const Entry current = open.top();
    open.pop();

    const std::size_t current_index = grid.index(current.col, current.row);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    if (current_index == goal_index) {
      std::vector<int> reversed_path;
      int index = static_cast<int>(current_index);
      while (index != -1) {
        reversed_path.push_back(index);
        index = came_from[static_cast<std::size_t>(index)];
      }
      return std::vector<int>(reversed_path.rbegin(), reversed_path.rend());
    }
    const int dir[8][2] = {{-1, -1}, {0, -1}, {1, -1}, {-1,  0}, {-1,  1}, {1,  0}, {1,  1}, {0,  1}};

    for (int neighbor = 0; neighbor < 8; ++neighbor) {
      const int dx = dir[neighbor][0];
      const int dy = dir[neighbor][1];
      const int neighbor_col = current.col + dx;
      const int neighbor_row = current.row + dy;
      const bool is_diagonal = (dx != 0 && dy != 0);

      if (isBlocked(grid, blocked, neighbor_col, neighbor_row)) {
        continue;
      }

      if (is_diagonal && (isBlocked(grid, blocked, current.col + dx, current.row) ||
          isBlocked(grid, blocked, current.col, current.row + dy))) {
        continue;
      }
      const int neighbor_index = grid.index(neighbor_col, neighbor_row);
      if (closed[neighbor_index]) {
        continue;
      }

      const double move_cost = grid.resolution * (is_diagonal ? std::sqrt(2.0) : 1.0);
      const double tentative_g = g_score[current_index] + move_cost;
      if (tentative_g < g_score[neighbor_index]) {
        g_score[neighbor_index] = tentative_g;
        came_from[neighbor_index] = static_cast<int>(current_index);
        open.push({tentative_g + heuristic(neighbor_col, neighbor_row),
                  neighbor_col, neighbor_row});
      }
    }
  }
  return {};
}

}  // namespace robonav_training
