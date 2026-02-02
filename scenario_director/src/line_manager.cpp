#include "scenario_director/line_manager.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace scenario_director {

RacingLine::RacingLine(std::vector<Waypoint> points)
: waypoints_(std::move(points)) {
  computeCumulativeDistance();
}

size_t RacingLine::size() const {
  return waypoints_.size();
}

const std::vector<Waypoint> &RacingLine::points() const {
  return waypoints_;
}

void RacingLine::computeCumulativeDistance() {
  cumulative_dist_.clear();
  cumulative_dist_.reserve(waypoints_.size());
  cumulative_dist_.push_back(0.0);

  double total = 0.0;
  for (size_t i = 1; i < waypoints_.size(); ++i) {
    const double dx = waypoints_[i].x - waypoints_[i - 1].x;
    const double dy = waypoints_[i].y - waypoints_[i - 1].y;
    total += std::sqrt(dx * dx + dy * dy);
    cumulative_dist_.push_back(total);
  }
  total_length_ = total;
}

int RacingLine::getNearestIndex(double x, double y) const {
  int best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const double dx = waypoints_[i].x - x;
    const double dy = waypoints_[i].y - y;
    const double dist = dx * dx + dy * dy;
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

Waypoint RacingLine::getWaypoint(int idx) const {
  if (waypoints_.empty()) {
    return {};
  }
  const int n = static_cast<int>(waypoints_.size());
  int wrapped = idx % n;
  if (wrapped < 0) {
    wrapped += n;
  }
  return waypoints_[static_cast<size_t>(wrapped)];
}

Waypoint RacingLine::getLookaheadPoint(double x, double y, double lookahead) const {
  if (waypoints_.empty()) {
    return {};
  }

  const int nearest_idx = getNearestIndex(x, y);
  const double nearest_dist = cumulative_dist_[static_cast<size_t>(nearest_idx)];
  double target_dist = nearest_dist + lookahead;

  if (total_length_ > 0.0 && target_dist > total_length_) {
    target_dist -= total_length_;
  }

  auto it = std::lower_bound(cumulative_dist_.begin(), cumulative_dist_.end(), target_dist);
  size_t target_idx = static_cast<size_t>(std::distance(cumulative_dist_.begin(), it));
  if (target_idx >= waypoints_.size()) {
    target_idx = waypoints_.size() - 1;
  }

  return waypoints_[target_idx];
}

void LineManager::setSpeedMultiplier(double multiplier) {
  speed_multiplier_ = std::max(0.0, multiplier);
}

void LineManager::setDefaultSpeed(double speed) {
  default_speed_ = std::max(0.0, speed);
}

void LineManager::loadOptimalLine(const std::string &csv_path) {
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open waypoint CSV: " + csv_path);
  }

  std::string line;
  std::getline(file, line); // header

  std::vector<Waypoint> points;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;

    while (std::getline(ss, token, ',')) {
      if (!token.empty()) {
        values.push_back(std::stod(token));
      }
    }
    if (values.size() < 2) {
      continue;
    }

    Waypoint wp;
    wp.x = values[0];
    wp.y = values[1];
    wp.yaw = values.size() >= 3 ? values[2] : 0.0;
    wp.speed = (values.size() >= 4 ? values[3] : default_speed_) * speed_multiplier_;
    points.push_back(wp);
  }

  if (points.empty()) {
    throw std::runtime_error("Waypoint CSV contains no valid points: " + csv_path);
  }

  optimal_line_ = std::make_shared<RacingLine>(std::move(points));
}

std::shared_ptr<RacingLine> LineManager::getLine(const std::string &name) const {
  if (name == "optimal") {
    return optimal_line_;
  }
  return nullptr;
}

}  // namespace scenario_director
