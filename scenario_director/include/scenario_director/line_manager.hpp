#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

namespace scenario_director {

struct Waypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

class RacingLine {
public:
  RacingLine() = default;
  RacingLine(std::vector<Waypoint> points, std::string name);

  const std::string &name() const;
  size_t size() const;
  double totalLength() const;

  int getNearestIndex(double x, double y) const;
  Waypoint getWaypoint(int idx) const;
  Waypoint getLookaheadPoint(double x, double y, double lookahead) const;

  const std::vector<Waypoint> &points() const;

private:
  void computeCumulativeDistance();

  std::string name_;
  std::vector<Waypoint> waypoints_;
  std::vector<double> cumulative_dist_;
  double total_length_ = 0.0;
};

class LineManager {
public:
  LineManager() = default;

  void setSpeedMultiplier(double multiplier);
  double speedMultiplier() const;

  void loadOptimalLine(const std::string &csv_path);
  void generateOffsetLines(double inside_offset, double outside_offset);

  std::shared_ptr<RacingLine> getLine(const std::string &name) const;
  std::shared_ptr<RacingLine> getCurrentLine() const;
  void setCurrentLine(const std::string &name);
  std::vector<std::string> getAllLineNames() const;

  void saveLines(const std::string &output_dir) const;

private:
  double speed_multiplier_ = 1.15;
  std::string current_line_name_ = "optimal";
  std::unordered_map<std::string, std::shared_ptr<RacingLine>> lines_;
};

}  // namespace scenario_director
