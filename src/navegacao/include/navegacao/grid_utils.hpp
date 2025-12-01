#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navegacao
{

struct GridCoordinate
{
  int row = 0;
  int col = 0;

  bool operator==(const GridCoordinate & other) const
  {
    return row == other.row && col == other.col;
  }
};

inline bool operator<(const GridCoordinate & lhs, const GridCoordinate & rhs)
{
  if (lhs.row == rhs.row) {
    return lhs.col < rhs.col;
  }
  return lhs.row < rhs.row;
}

struct GridMap
{
  std::vector<std::vector<char>> cells;

  size_t rows() const { return cells.size(); }
  size_t cols() const { return cells.empty() ? 0 : cells.front().size(); }

  char at(int r, int c) const { return cells.at(static_cast<size_t>(r)).at(static_cast<size_t>(c)); }
};

inline bool is_walkable(char value)
{
  return value == 'f' || value == 'r' || value == 't';
}

inline GridMap build_grid_from_response(const std::vector<std::string> & flattened,
                                        const std::vector<uint8_t> & shape)
{
  if (shape.size() != 2) {
    throw std::runtime_error("occupancy_grid_shape precisa ter 2 entradas (linhas, colunas)");
  }

  const size_t rows = shape[0];
  const size_t cols = shape[1];
  if (rows * cols != flattened.size()) {
    throw std::runtime_error("Dados do mapa incompletos");
  }

  GridMap grid;
  grid.cells.assign(rows, std::vector<char>(cols, 'b'));
  size_t idx = 0;
  for (size_t r = 0; r < rows; ++r) {
    for (size_t c = 0; c < cols; ++c) {
      grid.cells[r][c] = flattened[idx].empty() ? 'b' : flattened[idx][0];
      ++idx;
    }
  }
  return grid;
}

inline std::optional<GridCoordinate> find_symbol(const GridMap & grid, char symbol)
{
  for (size_t r = 0; r < grid.rows(); ++r) {
    for (size_t c = 0; c < grid.cols(); ++c) {
      if (grid.cells[r][c] == symbol) {
        return GridCoordinate{static_cast<int>(r), static_cast<int>(c)};
      }
    }
  }
  return std::nullopt;
}

inline double heuristic(const GridCoordinate & a, const GridCoordinate & b)
{
  return static_cast<double>(std::abs(a.row - b.row) + std::abs(a.col - b.col));
}

inline std::vector<GridCoordinate> reconstruct_path(
  const std::unordered_map<int, int> & came_from,
  GridCoordinate current,
  size_t cols)
{
  std::vector<GridCoordinate> path;
  auto key = [&](const GridCoordinate & coord) { return coord.row * static_cast<int>(cols) + coord.col; };
  path.push_back(current);
  auto it = came_from.find(key(current));
  while (it != came_from.end()) {
    int value = it->second;
    GridCoordinate prev{value / static_cast<int>(cols), value % static_cast<int>(cols)};
    path.push_back(prev);
    it = came_from.find(key(prev));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

inline bool run_a_star(const GridMap & grid,
                       const GridCoordinate & start,
                       const GridCoordinate & goal,
                       std::vector<GridCoordinate> & path)
{
  struct Node
  {
    GridCoordinate coord{};
    double g = 0.0;
    double f = 0.0;
  };

  auto key = [&](const GridCoordinate & coord) { return coord.row * static_cast<int>(grid.cols()) + coord.col; };

  auto cmp = [](const Node & lhs, const Node & rhs) { return lhs.f > rhs.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open_set(cmp);
  open_set.push(Node{start, 0.0, heuristic(start, goal)});

  std::unordered_map<int, double> g_score;
  g_score[key(start)] = 0.0;

  std::unordered_map<int, int> came_from;

  const std::vector<GridCoordinate> moves{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  while (!open_set.empty()) {
    Node current = open_set.top();
    open_set.pop();

    if (current.coord == goal) {
      path = reconstruct_path(came_from, current.coord, grid.cols());
      return true;
    }

    for (const auto & move : moves) {
      GridCoordinate neighbor{current.coord.row + move.row, current.coord.col + move.col};
      if (neighbor.row < 0 || neighbor.col < 0 ||
          neighbor.row >= static_cast<int>(grid.rows()) ||
          neighbor.col >= static_cast<int>(grid.cols())) {
        continue;
      }
      if (!is_walkable(grid.cells[neighbor.row][neighbor.col]) && grid.cells[neighbor.row][neighbor.col] != 't') {
        continue;
      }

      double tentative_g = current.g + 1.0;
      int neighbor_key = key(neighbor);
      if (!g_score.count(neighbor_key) || tentative_g < g_score[neighbor_key]) {
        came_from[neighbor_key] = key(current.coord);
        g_score[neighbor_key] = tentative_g;
        double f_score = tentative_g + heuristic(neighbor, goal);
        open_set.push(Node{neighbor, tentative_g, f_score});
      }
    }
  }

  return false;
}

inline std::string direction_from_step(const GridCoordinate & from, const GridCoordinate & to)
{
  int dr = to.row - from.row;
  int dc = to.col - from.col;
  if (dr == -1 && dc == 0) { return "up"; }
  if (dr == 1 && dc == 0) { return "down"; }
  if (dr == 0 && dc == -1) { return "left"; }
  if (dr == 0 && dc == 1) { return "right"; }
  return "";
}

}  // namespace navegacao

