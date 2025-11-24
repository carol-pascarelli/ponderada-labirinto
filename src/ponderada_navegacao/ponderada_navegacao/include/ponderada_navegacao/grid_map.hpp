#ifndef GRID_MAP_HPP
#define GRID_MAP_HPP

#include <vector>
#include <string>

struct Pos {
  int r;
  int c;
};

enum CellType : int {
  WALL = 1,
  FREE = 0,
  ROBOT = 2,
  TARGET = 3,
  UNKNOWN = -1
};

class GridMap {
public:
  std::vector<std::string> raw; // each string is a row of chars like 'b','f','r','t'
  int rows{0}, cols{0};

  void build_from_flat_strvec(const std::vector<std::string> &flat, int R, int C) {
    rows = R; cols = C;
    raw = flat;
  }

  bool is_free(int r, int c) const {
    if (r<0 || c<0 || r>=rows || c>=cols) return false;
    char ch = raw[r][c];
    return ch != 'b';
  }
};

#endif
