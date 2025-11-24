#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "grid_map.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <algorithm>

struct PathNode {
  int r,c;
};

inline int manhattan(const PathNode &a, const PathNode &b) {
  return abs(a.r - b.r) + abs(a.c - b.c);
}

// returns vector of PathNode from start (inclusive) to goal (inclusive)
// empty if no path
std::vector<PathNode> astar_path(const GridMap &map, const PathNode &start, const PathNode &goal) {
  int R = map.rows, C = map.cols;
  auto encode = [&](int r, int c){ return r * C + c; };

  struct Item {
    int f,g;
    int r,c;
  };
  struct Cmp { bool operator()(const Item &a, const Item &b) const { return a.f > b.f; } };
  std::priority_queue<Item, std::vector<Item>, Cmp> pq;

  std::unordered_map<int,int> gscore;
  std::unordered_map<int,int> parent; // encode(child) -> encode(parent)

  int s = encode(start.r, start.c);
  int t = encode(goal.r, goal.c);

  pq.push({manhattan(start,goal), 0, start.r, start.c});
  gscore[s] = 0;
  parent[s] = -1;

  int dr[4] = {-1,1,0,0};
  int dc[4] = {0,0,-1,1};

  while(!pq.empty()) {
    auto it = pq.top(); pq.pop();
    int r = it.r, c = it.c;
    int curenc = encode(r,c);
    if (curenc == t) break;

    for(int k=0;k<4;k++){
      int nr = r + dr[k], nc = c + dc[k];
      if (nr<0 || nc<0 || nr>=R || nc>=C) continue;
      if (!map.is_free(nr,nc)) continue;
      int neigh = encode(nr,nc);
      int tentative = gscore[curenc] + 1;
      if (!gscore.count(neigh) || tentative < gscore[neigh]) {
        gscore[neigh] = tentative;
        parent[neigh] = curenc;
        PathNode n{nr,nc};
        pq.push({tentative + manhattan(n, goal), tentative, nr, nc});
      }
    }
  }

  if (!parent.count(t)) return {};
  std::vector<PathNode> path;
  int cur = t;
  while(cur != -1) {
    int rr = cur / C;
    int cc = cur % C;
    path.push_back({rr,cc});
    cur = parent[cur];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

#endif
