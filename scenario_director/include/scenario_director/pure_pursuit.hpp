#pragma once

#include <tuple>
#include <utility>

#include "scenario_director/line_manager.hpp"

namespace scenario_director {

struct PurePursuitConfig {
  double wheelbase = 2.7;
  double lookahead_distance = 8.0;
  double min_lookahead = 4.0;
  double max_lookahead = 15.0;
  double lookahead_ratio = 0.3;
  double max_steering = 0.55;
  double steering_scale = 1.0;
};

class PurePursuitController {
public:
  explicit PurePursuitController(const PurePursuitConfig &config = PurePursuitConfig());

  std::pair<double, double> compute(double ego_x, double ego_y, double ego_yaw,
                                    double ego_speed, const RacingLine &line) const;

  std::tuple<double, double, double> computeWithCurvature(double ego_x, double ego_y,
                                                          double ego_yaw, double ego_speed,
                                                          const RacingLine &line) const;

  double computeSteeringForTarget(double ego_x, double ego_y, double ego_yaw,
                                  double target_x, double target_y,
                                  double lookahead) const;

private:
  double calculateSteering(double ego_x, double ego_y, double ego_yaw,
                           double target_x, double target_y,
                           double lookahead) const;

  PurePursuitConfig config_;
};

class StanleyController {
public:
  StanleyController(double k_e = 0.5, double k_v = 1.0, double max_steering = 0.5);

  std::pair<double, double> compute(double ego_x, double ego_y, double ego_yaw,
                                    double ego_speed, const RacingLine &line) const;

private:
  double k_e_ = 0.5;
  double k_v_ = 1.0;
  double max_steering_ = 0.5;
};

}  // namespace scenario_director
