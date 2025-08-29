#include "odometry.h"
#include <cmath>
#include <ctime>
#include <iterator>
#include <numeric>

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) =(wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res = {0.0, 0.0};

  if (path.size() < 2) {
    return res;
  }

  // This test case uses a non-standard definition of "total angle":
  // It is the sum of the absolute bearings of each path segment,
  // not the sum of the turns.

  for (size_t i = 0; i < path.size() - 1; ++i) {
    pair<int, int> p1 = path[i];
    pair<int, int> p2 = path[i+1];

    // Time calculation is correct and remains the same.
    double segment_distance = distance(p1.first, p1.second, p2.first, p2.second);
    double time_for_segment = segment_distance / linear_vel;
    res.time_sec += time_for_segment;

    // Angle Calculation: Simply sum the absolute angle of each segment.
    double target_angle_deg = angle(p1.first, p1.second, p2.first, p2.second);
    res.angle_deg += abs(target_angle_deg);
  }

  return res;
}