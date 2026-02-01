#include "scenario_director/line_manager.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace scenario_director {

RacingLine::RacingLine(std::vector<Waypoint> points, std::string name)
: name_(std::move(name)), waypoints_(std::move(points)) {
  computeCumulativeDistance();
}

const std::string &RacingLine::name() const {
  return name_;
}

size_t RacingLine::size() const {
  return waypoints_.size();
}

const std::vector<Waypoint> &RacingLine::points() const {
  return waypoints_;
}

double RacingLine::totalLength() const {
  return total_length_;
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

  if (target_dist > total_length_) {
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
  speed_multiplier_ = multiplier;
}

double LineManager::speedMultiplier() const {
  return speed_multiplier_;
}

void LineManager::loadOptimalLine(const std::string &csv_path) {
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open optimal line CSV: " + csv_path);
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
    if (values.size() < 4) {
      continue;
    }

    Waypoint wp;
    wp.x = values[0];
    wp.y = values[1];
    wp.yaw = values[2];
    wp.speed = values[3] * speed_multiplier_;
    points.push_back(wp);
  }

  lines_["optimal"] = std::make_shared<RacingLine>(std::move(points), "optimal");
}

void LineManager::generateOffsetLines(double inside_offset, double outside_offset) {
  auto optimal_it = lines_.find("optimal");
  if (optimal_it == lines_.end()) {
    throw std::runtime_error("Optimal line must be loaded before generating offsets");
  }

  const auto &optimal = optimal_it->second->points();
  std::vector<Waypoint> inside_points;
  std::vector<Waypoint> outside_points;
  inside_points.reserve(optimal.size());
  outside_points.reserve(optimal.size());

  for (const auto &wp : optimal) {
    const double normal_x = -std::sin(wp.yaw);
    const double normal_y = std::cos(wp.yaw);

    Waypoint inside_wp = wp;
    inside_wp.x = wp.x + inside_offset * normal_x;
    inside_wp.y = wp.y + inside_offset * normal_y;

    Waypoint outside_wp = wp;
    outside_wp.x = wp.x + outside_offset * normal_x;
    outside_wp.y = wp.y + outside_offset * normal_y;

    inside_points.push_back(inside_wp);
    outside_points.push_back(outside_wp);
  }

  lines_["inside"] = std::make_shared<RacingLine>(std::move(inside_points), "inside");
  lines_["outside"] = std::make_shared<RacingLine>(std::move(outside_points), "outside");
}

std::shared_ptr<RacingLine> LineManager::getLine(const std::string &name) const {
  auto it = lines_.find(name);
  if (it == lines_.end()) {
    return nullptr;
  }
  return it->second;
}

std::shared_ptr<RacingLine> LineManager::getCurrentLine() const {
  return getLine(current_line_name_);
}

void LineManager::setCurrentLine(const std::string &name) {
  if (lines_.find(name) != lines_.end()) {
    current_line_name_ = name;
  }
}

std::vector<std::string> LineManager::getAllLineNames() const {
  std::vector<std::string> names;
  names.reserve(lines_.size());
  for (const auto &item : lines_) {
    names.push_back(item.first);
  }
  return names;
}

void LineManager::saveLines(const std::string &output_dir) const {
  std::filesystem::create_directories(output_dir);

  for (const auto &pair : lines_) {
    const std::string &name = pair.first;
    const auto &line = pair.second->points();

    std::filesystem::path out_path = std::filesystem::path(output_dir) / (name + "_line.csv");
    std::ofstream out(out_path);
    if (!out.is_open()) {
      continue;
    }

    out << "x,y,yaw,v\n";
    for (const auto &wp : line) {
      out << wp.x << "," << wp.y << "," << wp.yaw << "," << (wp.speed / speed_multiplier_) << "\n";
    }
  }
}

}  // namespace scenario_director
