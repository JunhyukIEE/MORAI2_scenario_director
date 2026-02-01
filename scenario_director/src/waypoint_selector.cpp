#include "scenario_director/waypoint_selector.hpp"

#include <cmath>
#include <iostream>

namespace scenario_director {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

WaypointSelector::WaypointSelector(std::shared_ptr<LineManager> line_manager,
                                   const SelectorConfig &config,
                                   std::shared_ptr<TrackBoundaryChecker> boundary_checker)
: line_manager_(std::move(line_manager)),
  config_(config),
  boundary_checker_(std::move(boundary_checker)) {}

std::pair<std::shared_ptr<RacingLine>, std::string> WaypointSelector::update(
    const VehicleState &ego,
    const std::shared_ptr<VehicleState> &opponent) {
  if (abort_cooldown_ > 0) {
    --abort_cooldown_;
  }

  if (!opponent) {
    transitionTo(DrivingState::NORMAL);
    return getLine("optimal");
  }

  const auto [rel_dist, rel_angle] = calcRelative(ego, *opponent);
  const bool collision_risk = checkCollisionRisk(ego, *opponent, rel_dist, rel_angle);

  if (collision_risk) {
    if (state_ == DrivingState::APPROACHING || state_ == DrivingState::OVERTAKING) {
      abortOvertake();
      return getLine("optimal");
    }
  }

  if (state_ == DrivingState::NORMAL) {
    if (!canSafelyOvertake(ego, *opponent, rel_dist, rel_angle)) {
      return getLine("optimal");
    }
  }

  const DrivingState new_state = determineState(ego, *opponent, rel_dist, rel_angle);
  if (new_state != state_) {
    ++state_counter_;
    if (state_counter_ >= state_threshold_) {
      transitionTo(new_state);
      state_counter_ = 0;
    }
  } else {
    state_counter_ = 0;
  }

  return selectLineByState(ego, *opponent, rel_dist, rel_angle);
}

DrivingState WaypointSelector::getState() const {
  return state_;
}

std::string WaypointSelector::getCurrentLineName() const {
  return current_line_;
}

bool WaypointSelector::isOvertaking() const {
  return state_ == DrivingState::APPROACHING ||
         state_ == DrivingState::OVERTAKING ||
         state_ == DrivingState::COMPLETING;
}

std::pair<bool, int> WaypointSelector::getAbortStatus() const {
  return {overtake_aborted_, abort_cooldown_};
}

void WaypointSelector::transitionTo(DrivingState new_state) {
  state_ = new_state;
  if (new_state == DrivingState::NORMAL) {
    current_line_ = "optimal";
    overtake_side_.clear();
  }
}

void WaypointSelector::abortOvertake() {
  state_ = DrivingState::NORMAL;
  current_line_ = "optimal";
  overtake_side_.clear();
  overtake_aborted_ = true;
  state_counter_ = 0;
}

std::pair<double, double> WaypointSelector::calcRelative(const VehicleState &ego,
                                                         const VehicleState &opponent) const {
  const double dx = opponent.x - ego.x;
  const double dy = opponent.y - ego.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  const double abs_angle = std::atan2(dy, dx);
  double rel_angle = abs_angle - ego.yaw;
  rel_angle = std::atan2(std::sin(rel_angle), std::cos(rel_angle));
  const double rel_angle_deg = rel_angle * 180.0 / kPi;

  return {dist, rel_angle_deg};
}

bool WaypointSelector::checkCollisionRisk(const VehicleState &ego, const VehicleState &opponent,
                                          double rel_dist, double rel_angle) const {
  const bool is_behind = std::abs(rel_angle) > config_.rear_danger_angle;
  if (!is_behind) {
    return false;
  }

  const double closing_speed = opponent.speed - ego.speed;
  if (closing_speed <= 0.0) {
    return false;
  }

  if (rel_dist > 0.0) {
    const double ttc = rel_dist / closing_speed;
    if (ttc < config_.ttc_threshold) {
      return true;
    }
  }

  if (rel_dist < config_.min_safe_gap) {
    return true;
  }

  return false;
}

bool WaypointSelector::canSafelyOvertake(const VehicleState &ego, const VehicleState &opponent,
                                         double rel_dist, double rel_angle) {
  if (abort_cooldown_ > 0) {
    return false;
  }

  if (overtake_aborted_) {
    overtake_aborted_ = false;
    abort_cooldown_ = 50; // ~2.5s @ 20Hz
    return false;
  }

  const bool is_ahead = std::abs(rel_angle) < config_.overtake_angle;
  if (!is_ahead) {
    return false;
  }

  const double speed_diff = ego.speed - opponent.speed;
  if (speed_diff < config_.min_overtake_speed_diff) {
    return false;
  }

  if (rel_dist < config_.min_safe_gap) {
    return false;
  }

  if (rel_dist > config_.overtake_distance) {
    return false;
  }

  return true;
}

DrivingState WaypointSelector::determineState(const VehicleState &ego, const VehicleState &opponent,
                                              double rel_dist, double rel_angle) const {
  const bool is_ahead = std::abs(rel_angle) < config_.overtake_angle;
  const bool is_behind = std::abs(rel_angle) > config_.complete_angle;

  const bool is_close = rel_dist < config_.overtake_distance;
  const bool is_very_close = rel_dist < config_.complete_distance;

  const double speed_diff = ego.speed - opponent.speed;
  const bool is_faster = speed_diff > config_.min_overtake_speed_diff;

  if (state_ == DrivingState::NORMAL) {
    if (is_ahead && is_close && is_faster) {
      return DrivingState::APPROACHING;
    }
  } else if (state_ == DrivingState::APPROACHING) {
    if (is_ahead && is_very_close) {
      return DrivingState::OVERTAKING;
    }
    if (!is_ahead || !is_close) {
      return DrivingState::NORMAL;
    }
  } else if (state_ == DrivingState::OVERTAKING) {
    if (is_behind && is_very_close) {
      return DrivingState::COMPLETING;
    }
    if (rel_dist > config_.safe_distance) {
      return DrivingState::NORMAL;
    }
  } else if (state_ == DrivingState::COMPLETING) {
    if (is_behind && rel_dist > config_.complete_distance) {
      return DrivingState::NORMAL;
    }
  }

  return state_;
}

std::pair<std::shared_ptr<RacingLine>, std::string> WaypointSelector::selectLineByState(
    const VehicleState &ego, const VehicleState &opponent,
    double rel_dist, double rel_angle) {
  (void)rel_dist;
  (void)rel_angle;

  if (state_ == DrivingState::NORMAL) {
    return getLine("optimal");
  }

  if (state_ == DrivingState::APPROACHING) {
    if (overtake_side_.empty()) {
      overtake_side_ = decideOvertakeSide(ego, opponent);
    }

    if (overtake_side_ == "optimal") {
      abortOvertake();
      return getLine("optimal");
    }

    return getLine(overtake_side_);
  }

  if (state_ == DrivingState::OVERTAKING) {
    return getLine(overtake_side_);
  }

  if (state_ == DrivingState::COMPLETING) {
    return getLine("optimal");
  }

  return getLine("optimal");
}

std::string WaypointSelector::decideOvertakeSide(const VehicleState &ego, const VehicleState &opponent) const {
  const double dx = opponent.x - ego.x;
  const double dy = opponent.y - ego.y;

  const double left_x = -std::sin(ego.yaw);
  const double left_y = std::cos(ego.yaw);

  const double dot = dx * left_x + dy * left_y;

  std::string preferred = dot > 0.0 ? "inside" : "outside";
  std::string alternative = dot > 0.0 ? "outside" : "inside";

  if (boundary_checker_) {
    const bool preferred_safe = isLineSafe(ego, preferred);
    const bool alternative_safe = isLineSafe(ego, alternative);

    if (preferred_safe) {
      return preferred;
    }
    if (alternative_safe) {
      return alternative;
    }
    return "optimal";
  }

  return preferred;
}

bool WaypointSelector::isLineSafe(const VehicleState &ego, const std::string &line_name) const {
  if (!boundary_checker_) {
    return true;
  }

  auto line = line_manager_->getLine(line_name);
  if (!line) {
    return false;
  }

  const int nearest_idx = line->getNearestIndex(ego.x, ego.y);
  const double check_distance = 20.0;
  const int check_points = static_cast<int>(check_distance / 0.5);

  for (int i = 0; i < check_points; ++i) {
    const int idx = (nearest_idx + i) % static_cast<int>(line->size());
    const Waypoint wp = line->getWaypoint(idx);
    if (!boundary_checker_->isDrivableWithMargin(wp.x, wp.y, config_.vehicle_width)) {
      return false;
    }
  }

  return true;
}

std::pair<std::shared_ptr<RacingLine>, std::string> WaypointSelector::getLine(const std::string &name) {
  current_line_ = name;
  return {line_manager_->getLine(name), name};
}

}  // namespace scenario_director
