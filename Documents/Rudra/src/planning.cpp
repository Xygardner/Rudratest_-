#include "planning.h"
#include <cmath>
#include <vector>
#include <queue>      
#include <map>        
#include <limits>     
#include <algorithm>

using namespace std;

Planner::Planner(const vector<vector<bool>> &grid) : grid(grid) {
  rows = grid.size();
  cols = grid[0].size();
}

bool Planner::isvalid(int x, int y) const {
  return (x >= 0 && x < rows && y >= 0 && y < cols && !grid[x][y]);
}

double Planner::heuristic(int x1, int y1, int x2, int y2) const {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

vector<pair<int, int>> Planner::pathplanning(pair<int, int> start,
                                             pair<int, int> goal) {
  /* Implement Path Planning logic here */

  struct Node {
    pair<int, int> pos;
    double fScore;
    bool operator>(const Node& other) const {
        return fScore > other.fScore;
    }
  };

  priority_queue<Node, vector<Node>, greater<Node>> openSet;
  map<pair<int, int>, pair<int, int>> cameFrom;

  vector<vector<double>> gScore(rows, vector<double>(cols, numeric_limits<double>::infinity()));
  gScore[start.first][start.second] = 0;

  vector<vector<double>> fScore(rows, vector<double>(cols, numeric_limits<double>::infinity()));
  fScore[start.first][start.second] = heuristic(start.first, start.second, goal.first, goal.second);

  openSet.push({start, fScore[start.first][start.second]});

  while (!openSet.empty()) {
      pair<int, int> current = openSet.top().pos;
      openSet.pop();

      if (current == goal) {
          vector<pair<int, int>> path;
          path.push_back(current);
          while (cameFrom.count(current)) {
              current = cameFrom[current];
              path.push_back(current);
          }
          reverse(path.begin(), path.end());
          return path;
      }

      // UPDATED: Define 8 directions for neighbor search (including diagonals)
      int dx[] = {-1, 1, 0, 0, -1, -1, 1, 1};
      int dy[] = {0, 0, -1, 1, -1, 1, -1, 1};

      for (int i = 0; i < 8; ++i) { // Loop through all 8 neighbors
          pair<int, int> neighbor = {current.first + dx[i], current.second + dy[i]};

          if (isvalid(neighbor.first, neighbor.second)) {
              // UPDATED: Cost is sqrt(2) for diagonals, 1 for cardinal directions
              double move_cost = (dx[i] != 0 && dy[i] != 0) ? sqrt(2.0) : 1.0;
              double tentative_gScore = gScore[current.first][current.second] + move_cost;

              if (tentative_gScore < gScore[neighbor.first][neighbor.second]) {
                  cameFrom[neighbor] = current;
                  gScore[neighbor.first][neighbor.second] = tentative_gScore;
                  fScore[neighbor.first][neighbor.second] = tentative_gScore + heuristic(neighbor.first, neighbor.second, goal.first, goal.second);
                  openSet.push({neighbor, fScore[neighbor.first][neighbor.second]});
              }
          }
      }
  }
  
  return {};
}