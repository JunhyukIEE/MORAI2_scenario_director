#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "scenario_director/track_boundary.hpp"

namespace scenario_director {

// -----------------------------
// Configuration structures
// -----------------------------
struct VehicleParams {
  double wheelbase = 2.7;
  double max_steer = 0.489;      // radians (~28 deg)
  double max_accel = 3.5;
  double max_decel = 7.0;
  double max_lat_accel = 8.5;
};

struct PlannerWeights {
  double w_progress = 8.0;
  double w_speed = 1.5;
  double w_overtake = 60.0;
  double w_collision = 20000.0;
  double w_offroad = 4000.0;
  double w_smooth = 8.0;
  double w_curv = 2.0;
  double w_far_from_ref = 2.0;
};

struct LocalPlannerConfig {
  VehicleParams vehicle;
  PlannerWeights weights;

  double ego_length = 4.0;
  double ego_width = 1.7;
  double opp_length = 4.0;
  double opp_width = 1.7;

  double dt = 0.1;
  double horizon_s = 4.0;
  double path_ahead_m = 70.0;

  std::vector<double> lateral_offsets = {-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0};
  std::vector<double> speed_scales = {0.85, 1.0, 1.10, 1.20};

  int offroad_check_stride = 2;
  double overtake_margin_s = 6.0;
  double near_pass_bonus_dist = 3.0;

  // Hysteresis: penalty for changing lateral offset direction
  double lat_change_penalty = 15.0;  // Penalty per meter of offset change
};

// -----------------------------
// Trajectory point
// -----------------------------
struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
};

// -----------------------------
// Planning result
// -----------------------------
struct PlanResult {
  double reward = 0.0;
  double lat_offset = 0.0;
  double speed_scale = 0.0;
  std::vector<TrajectoryPoint> trajectory;
  std::vector<std::pair<double, double>> candidate_path;  // (x, y)
  std::vector<double> v_desired;

  struct DebugCandidate {
    double reward;
    double lat_offset;
    double speed_scale;
  };
  std::vector<DebugCandidate> debug_top;
};

// -----------------------------
// Reference Line (record line)
// -----------------------------
class ReferenceLine {
public:
  ReferenceLine(const std::string& waypoints_csv, bool loop = true);

  void load();
  size_t size() const { return N_; }
  double trackLength() const { return track_len_; }

  int nearestIndex(double x, double y) const;
  double nearestS(double x, double y) const;

  std::vector<int> getSegmentIndices(int idx0, double length_m) const;

  void getShiftedLine(const std::vector<int>& indices, double offset_m,
                      std::vector<double>& out_x, std::vector<double>& out_y) const;

  // Direct access
  const std::vector<double>& x() const { return x_; }
  const std::vector<double>& y() const { return y_; }
  const std::vector<double>& yaw() const { return yaw_; }
  const std::vector<double>& v_ref() const { return v_ref_; }
  const std::vector<double>& s() const { return s_; }
  const std::vector<double>& kappa() const { return kappa_; }

private:
  std::string waypoints_csv_;
  bool loop_;
  size_t N_ = 0;
  double track_len_ = 0.0;

  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> v_ref_;
  std::vector<double> s_;
  std::vector<double> kappa_;
};

// -----------------------------
// Opponent Predictor
// -----------------------------
class OpponentPredictor {
public:
  explicit OpponentPredictor(std::shared_ptr<ReferenceLine> ref);

  void predictStates(double opp_x, double opp_y, double opp_v,
                     double horizon_s, double dt,
                     std::vector<double>& out_x,
                     std::vector<double>& out_y,
                     std::vector<double>& out_yaw) const;

private:
  std::shared_ptr<ReferenceLine> ref_;
};

// -----------------------------
// Local Path Planner
// -----------------------------
class LocalPathPlanner {
public:
  LocalPathPlanner(const std::string& map_dir,
                   const std::string& map_yaml,
                   const std::string& waypoints_csv,
                   const LocalPlannerConfig& config = LocalPlannerConfig());

  PlanResult plan(double ego_x, double ego_y, double ego_yaw, double ego_v,
                  double opp_x, double opp_y, double opp_v);

  std::shared_ptr<ReferenceLine> getReferenceLine() const { return ref_; }
  const LocalPlannerConfig& config() const { return config_; }

private:
  // Utility functions
  static double wrapPi(double angle);
  static std::array<std::array<double, 2>, 4> obbCorners(double x, double y, double yaw,
                                                          double L, double W);
  static bool obbIntersect(const std::array<std::array<double, 2>, 4>& c1,
                           const std::array<std::array<double, 2>, 4>& c2);

  // Path following rollout
  std::vector<TrajectoryPoint> rolloutFollowPath(
      double x0, double y0, double yaw0, double v0,
      const std::vector<double>& path_x,
      const std::vector<double>& path_y,
      const std::vector<double>& v_des) const;

  // Reward computation
  double computeReward(const std::vector<TrajectoryPoint>& traj,
                       const std::vector<double>& cand_kappa,
                       double lat_offset,
                       const std::vector<double>& opp_x,
                       const std::vector<double>& opp_y,
                       const std::vector<double>& opp_yaw) const;

  // Curvature computation
  static std::vector<double> computeCurvature(const std::vector<double>& x,
                                               const std::vector<double>& y);

  // s-delta for loop
  double sDeltaLoop(double s_from, double s_to) const;

  LocalPlannerConfig config_;
  TrackBoundaryChecker checker_;
  std::shared_ptr<ReferenceLine> ref_;
  std::shared_ptr<OpponentPredictor> opp_pred_;

  int T_;  // number of timesteps in horizon

  // Hysteresis state
  double prev_lat_offset_ = 0.0;
  bool has_prev_offset_ = false;
};

}  // namespace scenario_director
