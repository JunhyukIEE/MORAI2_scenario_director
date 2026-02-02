#pragma once

#include <memory>
#include <string>
#include <vector>

namespace scenario_director {

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

class RacingLine {
public:
  explicit RacingLine(std::vector<Waypoint> points);

  size_t size() const;
  const std::vector<Waypoint> &points() const;
  int getNearestIndex(double x, double y) const;
  Waypoint getWaypoint(int idx) const;
  Waypoint getLookaheadPoint(double x, double y, double lookahead) const;

private:
  void computeCumulativeDistance();

  std::vector<Waypoint> waypoints_;
  std::vector<double> cumulative_dist_;
  double total_length_ = 0.0;
};

class LineManager {
public:
  void setSpeedMultiplier(double multiplier);
  void setDefaultSpeed(double speed);
  void loadOptimalLine(const std::string &csv_path);
  std::shared_ptr<RacingLine> getLine(const std::string &name) const;

private:
  double speed_multiplier_ = 1.0;
  double default_speed_ = 5.0;
  std::shared_ptr<RacingLine> optimal_line_;
};

}  // namespace scenario_director
