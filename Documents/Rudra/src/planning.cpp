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

  // Step 1: Define a local helper struct for the priority queue
  struct Node {
    pair<int, int> pos;
    double fScore;
    // Overload operator for the min-heap behavior
    bool operator>(const Node& other) const {
        return fScore > other.fScore;
    }
  };

  // Step 2: Initialization of A* data structures
  priority_queue<Node, vector<Node>, greater<Node>> openSet;
  map<pair<int, int>, pair<int, int>> cameFrom;

  vector<vector<double>> gScore(rows, vector<double>(cols, numeric_limits<double>::infinity()));
  gScore[start.first][start.second] = 0;

  vector<vector<double>> fScore(rows, vector<double>(cols, numeric_limits<double>::infinity()));
  fScore[start.first][start.second] = heuristic(start.first, start.second, goal.first, goal.second);

  openSet.push({start, fScore[start.first][start.second]});

  // Step 3: The Search Loop
  while (!openSet.empty()) {
      pair<int, int> current = openSet.top().pos;
      openSet.pop();

      // Check if we've reached the goal
      if (current == goal) {
          // Step 4: Inline Path Reconstruction
          vector<pair<int, int>> path;
          path.push_back(current);
          while (cameFrom.count(current)) {
              current = cameFrom[current];
              path.push_back(current);
          }
          reverse(path.begin(), path.end());
          return path; // Path found and returned
      }

      // Explore neighbors (up, down, left, right)
      int dx[] = {-1, 1, 0, 0};
      int dy[] = {0, 0, -1, 1};

      for (int i = 0; i < 4; ++i) {
          pair<int, int> neighbor = {current.first + dx[i], current.second + dy[i]};

          if (isvalid(neighbor.first, neighbor.second)) {
              double tentative_gScore = gScore[current.first][current.second] + 1;

              if (tentative_gScore < gScore[neighbor.first][neighbor.second]) {
                  // This path to the neighbor is better. Record it.
                  cameFrom[neighbor] = current;
                  gScore[neighbor.first][neighbor.second] = tentative_gScore;
                  fScore[neighbor.first][neighbor.second] = tentative_gScore + heuristic(neighbor.first, neighbor.second, goal.first, goal.second);
                  openSet.push({neighbor, fScore[neighbor.first][neighbor.second]});
              }
          }
      }
  }

  // If the loop finishes and we never reached the goal, no path exists.
  return {}; // Return an empty path
}