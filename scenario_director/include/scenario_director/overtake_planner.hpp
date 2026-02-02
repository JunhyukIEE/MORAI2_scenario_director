#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "scenario_director/track_boundary.hpp"

namespace scenario_director {

struct OvertakeConfig {
  double safety_margin_m = 0.25;
  double alat_max = 6.0;
  double amax = 2.0;
  double bmax = 3.0;
  int num_candidates_per_corner = 7;
  double curvature_threshold_percentile = 80.0;
  double raycast_step_m = 0.05;
  double raycast_max_m = 6.0;
  double offtrack_max_ratio = 0.15;
  double clamp_max_ratio = 0.25;
  double corner_buffer_m = 2.0;
  int smooth_curvature_window = 11;
  double min_corner_length_m = 5.0;
  double min_inside_offset_m = 0.4;
  double family_a_outside_frac = 0.8;
  double family_a_inside_frac = 0.35;
  double family_b_inside_frac = 0.8;
  double family_c_feint_frac = 0.4;
  double family_c_switch_frac = 0.5;
  double speed_entry_scale = 0.85;
  double speed_exit_scale = 1.08;
  double late_brake_distance_m = 3.5;
  double dummy_feint_distance_m = 2.0;
  int debug_plot_dpi = 140;
};

struct TrackLimits {
  std::vector<double> left;
  std::vector<double> right;
};

struct CornerSegment {
  int corner_id = 0;
  int start_idx = 0;
  int end_idx = 0;
  int apex_idx = 0;
  int turn_dir = 0;
};

struct Candidate {
  int candidate_id = 0;
  int corner_id = 0;
  char family = 'A';
  std::vector<double> l;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> v;
  std::vector<std::string> phase;
  double quality_score = 0.0;
};

class OvertakePlanner {
public:
  OvertakePlanner(const std::string &map_dir,
                  const std::string &map_yaml,
                  const std::string &waypoints_path,
                  const OvertakeConfig &config);

  void generate(const std::string &out_dir);

private:
  void loadWaypoints();
  void computeArcLength();
  void computeYawIfMissing();
  void computeCurvature();
  void smoothCurvature();
  void computeBaseSpeed();

  TrackLimits estimateTrackLimits() const;
  std::vector<CornerSegment> detectCorners() const;
  std::vector<Candidate> generateCandidates(const TrackLimits &limits,
                                            const std::vector<CornerSegment> &corners) const;
  void applySpeedProfile(Candidate &cand, const CornerSegment &corner) const;
  std::pair<double, double> validateCandidate(const Candidate &cand) const;

  void writeTrackLimits(const TrackLimits &limits, const std::string &out_dir) const;
  void writeCorners(const std::vector<CornerSegment> &corners,
                    const TrackLimits &limits,
                    const std::string &out_dir) const;
  void writeCandidates(const std::vector<Candidate> &candidates, const std::string &out_dir) const;
  void writePlots(const std::vector<Candidate> &candidates,
                  const std::vector<CornerSegment> &corners,
                  const std::string &out_dir) const;

  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> v_raw_;
  std::vector<double> s_;
  std::vector<double> kappa_;
  std::vector<double> v_ref_;

  std::string waypoints_path_;
  OvertakeConfig config_;
  TrackBoundaryChecker checker_;
};

}  // namespace scenario_director
