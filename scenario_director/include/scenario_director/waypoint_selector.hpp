#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/track_boundary.hpp"

namespace scenario_director {

enum class DrivingState {
  NORMAL,
  APPROACHING,
  OVERTAKING,
  COMPLETING
};

struct VehicleState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double speed = 0.0;
};

struct SelectorConfig {
  double overtake_distance = 15.0;
  double overtake_angle = 30.0;
  double min_overtake_speed_diff = 3.0;

  double complete_distance = 10.0;
  double complete_angle = 150.0;
  double safe_distance = 20.0;

  double min_safe_gap = 5.0;
  double ttc_threshold = 2.0;
  double rear_danger_angle = 120.0;

  double vehicle_width = 1.8;
};

class WaypointSelector {
public:
  WaypointSelector(std::shared_ptr<LineManager> line_manager,
                   const SelectorConfig &config,
                   std::shared_ptr<TrackBoundaryChecker> boundary_checker = nullptr);

  std::pair<std::shared_ptr<RacingLine>, std::string> update(
      const VehicleState &ego,
      const std::shared_ptr<VehicleState> &opponent);

  DrivingState getState() const;
  std::string getCurrentLineName() const;
  bool isOvertaking() const;
  std::pair<bool, int> getAbortStatus() const;

private:
  void transitionTo(DrivingState new_state);
  void abortOvertake();

  std::pair<double, double> calcRelative(const VehicleState &ego, const VehicleState &opponent) const;
  bool checkCollisionRisk(const VehicleState &ego, const VehicleState &opponent,
                          double rel_dist, double rel_angle) const;
  bool canSafelyOvertake(const VehicleState &ego, const VehicleState &opponent,
                         double rel_dist, double rel_angle);
  DrivingState determineState(const VehicleState &ego, const VehicleState &opponent,
                              double rel_dist, double rel_angle) const;

  std::pair<std::shared_ptr<RacingLine>, std::string> selectLineByState(
      const VehicleState &ego, const VehicleState &opponent,
      double rel_dist, double rel_angle);

  std::string decideOvertakeSide(const VehicleState &ego, const VehicleState &opponent) const;
  bool isLineSafe(const VehicleState &ego, const std::string &line_name) const;

  std::pair<std::shared_ptr<RacingLine>, std::string> getLine(const std::string &name);

  std::shared_ptr<LineManager> line_manager_;
  SelectorConfig config_;
  std::shared_ptr<TrackBoundaryChecker> boundary_checker_;

  DrivingState state_ = DrivingState::NORMAL;
  std::string current_line_ = "optimal";
  std::string overtake_side_;

  int state_counter_ = 0;
  int state_threshold_ = 5;

  bool overtake_aborted_ = false;
  int abort_cooldown_ = 0;
};

}  // namespace scenario_director
