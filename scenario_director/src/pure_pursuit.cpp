#include "scenario_director/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace scenario_director {

PurePursuitController::PurePursuitController(const PurePursuitConfig &config)
: config_(config) {}

std::pair<double, double> PurePursuitController::compute(double ego_x, double ego_y,
                                                         double ego_yaw, double ego_speed,
                                                         const RacingLine &line) const {
  double lookahead = config_.lookahead_distance + config_.lookahead_ratio * ego_speed;
  lookahead = std::clamp(lookahead, config_.min_lookahead, config_.max_lookahead);

  Waypoint target = line.getLookaheadPoint(ego_x, ego_y, lookahead);

  double steering = calculateSteering(ego_x, ego_y, ego_yaw, target.x, target.y, lookahead);
  steering *= config_.steering_scale;
  steering = std::clamp(steering, -config_.max_steering, config_.max_steering);

  return {steering, target.speed};
}

std::tuple<double, double, double> PurePursuitController::computeWithCurvature(
    double ego_x, double ego_y, double ego_yaw, double ego_speed, const RacingLine &line) const {
  auto [steering, target_speed] = compute(ego_x, ego_y, ego_yaw, ego_speed, line);
  const double curvature = std::tan(steering) / config_.wheelbase;
  return {steering, target_speed, curvature};
}

double PurePursuitController::calculateSteering(double ego_x, double ego_y, double ego_yaw,
                                                double target_x, double target_y,
                                                double lookahead) const {
  const double dx = target_x - ego_x;
  const double dy = target_y - ego_y;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < 0.1) {
    return 0.0;
  }

  const double target_angle = std::atan2(dy, dx);
  double alpha = target_angle - ego_yaw;
  alpha = std::atan2(std::sin(alpha), std::cos(alpha));

  return std::atan2(2.0 * config_.wheelbase * std::sin(alpha), lookahead);
}

StanleyController::StanleyController(double k_e, double k_v, double max_steering)
: k_e_(k_e), k_v_(k_v), max_steering_(max_steering) {}

std::pair<double, double> StanleyController::compute(double ego_x, double ego_y,
                                                     double ego_yaw, double ego_speed,
                                                     const RacingLine &line) const {
  const int nearest_idx = line.getNearestIndex(ego_x, ego_y);
  const Waypoint target = line.getWaypoint(nearest_idx);
  const Waypoint next_target = line.getWaypoint(nearest_idx + 1);

  const double path_yaw = std::atan2(next_target.y - target.y, next_target.x - target.x);
  double yaw_error = path_yaw - ego_yaw;
  yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

  const double dx = ego_x - target.x;
  const double dy = ego_y - target.y;
  const double cross_track = -dx * std::sin(path_yaw) + dy * std::cos(path_yaw);

  double steering = yaw_error + std::atan2(k_e_ * cross_track, k_v_ + ego_speed);
  steering = std::clamp(steering, -max_steering_, max_steering_);

  return {steering, target.speed};
}

}  // namespace scenario_director
