#include "scenario_director/local_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>

namespace scenario_director {

namespace {

// Compute arc length along polyline
std::vector<double> computeArcLength(const std::vector<double>& x,
                                      const std::vector<double>& y) {
  size_t N = x.size();
  std::vector<double> s(N, 0.0);
  for (size_t i = 1; i < N; ++i) {
    double dx = x[i] - x[i - 1];
    double dy = y[i] - y[i - 1];
    s[i] = s[i - 1] + std::hypot(dx, dy);
  }
  return s;
}

// Compute curvature from polyline using Menger curvature
std::vector<double> computePolylineCurvature(const std::vector<double>& x,
                                              const std::vector<double>& y) {
  size_t M = x.size();
  std::vector<double> kappa(M, 0.0);
  if (M < 3) {
    return kappa;
  }

  for (size_t i = 1; i + 1 < M; ++i) {
    double ax = x[i] - x[i - 1];
    double ay = y[i] - y[i - 1];
    double bx = x[i + 1] - x[i];
    double by = y[i + 1] - y[i];

    double cross = ax * by - ay * bx;
    double na = std::hypot(ax, ay) + 1e-9;
    double nb = std::hypot(bx, by) + 1e-9;
    double chord = std::hypot(x[i + 1] - x[i - 1], y[i + 1] - y[i - 1]) + 1e-9;

    kappa[i] = 2.0 * cross / (na * nb * chord);
  }
  kappa[0] = kappa[1];
  kappa[M - 1] = kappa[M - 2];
  return kappa;
}

}  // namespace

// -----------------------------
// ReferenceLine implementation
// -----------------------------
ReferenceLine::ReferenceLine(const std::string& waypoints_csv, bool loop)
    : waypoints_csv_(waypoints_csv), loop_(loop) {}

void ReferenceLine::load() {
  std::ifstream file(waypoints_csv_);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open waypoints: " + waypoints_csv_);
  }

  std::string line;
  bool header_checked = false;

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    std::stringstream ss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (!header_checked) {
      header_checked = true;
      if (!tokens.empty()) {
        try {
          std::stod(tokens[0]);
        } catch (...) {
          continue;  // Skip header
        }
      }
    }

    if (tokens.size() < 4) continue;

    x_.push_back(std::stod(tokens[0]));
    y_.push_back(std::stod(tokens[1]));
    yaw_.push_back(std::stod(tokens[2]));
    v_ref_.push_back(std::stod(tokens[3]));
  }

  if (x_.empty()) {
    throw std::runtime_error("Waypoints contain no data: " + waypoints_csv_);
  }

  N_ = x_.size();

  // Compute arc length
  s_ = computeArcLength(x_, y_);
  if (loop_ && N_ > 1) {
    double closing = std::hypot(x_[0] - x_[N_ - 1], y_[0] - y_[N_ - 1]);
    track_len_ = s_[N_ - 1] + closing;
  } else {
    track_len_ = s_[N_ - 1];
  }

  // Compute curvature
  kappa_ = computePolylineCurvature(x_, y_);
}

int ReferenceLine::nearestIndex(double x, double y) const {
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = 0;

  for (size_t i = 0; i < N_; ++i) {
    double dx = x_[i] - x;
    double dy = y_[i] - y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = static_cast<int>(i);
    }
  }
  return min_idx;
}

double ReferenceLine::nearestS(double x, double y) const {
  int idx = nearestIndex(x, y);
  return s_[idx];
}

std::vector<int> ReferenceLine::getSegmentIndices(int idx0, double length_m) const {
  std::vector<int> idxs;
  idxs.push_back(idx0);

  double acc = 0.0;
  int i = idx0;
  int N = static_cast<int>(N_);

  while (acc < length_m && idxs.size() < N_) {
    int j = loop_ ? (i + 1) % N : std::min(i + 1, N - 1);
    double dx = x_[j] - x_[i];
    double dy = y_[j] - y_[i];
    acc += std::hypot(dx, dy);
    idxs.push_back(j);
    if (!loop_ && j == N - 1) break;
    i = j;
  }

  return idxs;
}

void ReferenceLine::getShiftedLine(const std::vector<int>& indices, double offset_m,
                                   std::vector<double>& out_x, std::vector<double>& out_y) const {
  size_t M = indices.size();
  out_x.resize(M);
  out_y.resize(M);

  for (size_t i = 0; i < M; ++i) {
    int idx = indices[i];
    double nx = -std::sin(yaw_[idx]);
    double ny = std::cos(yaw_[idx]);
    out_x[i] = x_[idx] + offset_m * nx;
    out_y[i] = y_[idx] + offset_m * ny;
  }
}

// -----------------------------
// OpponentPredictor implementation
// -----------------------------
OpponentPredictor::OpponentPredictor(std::shared_ptr<ReferenceLine> ref)
    : ref_(std::move(ref)) {}

void OpponentPredictor::predictStates(double opp_x, double opp_y, double opp_v,
                                       double horizon_s, double dt,
                                       std::vector<double>& out_x,
                                       std::vector<double>& out_y,
                                       std::vector<double>& out_yaw) const {
  int T = static_cast<int>(horizon_s / dt);
  out_x.resize(T);
  out_y.resize(T);
  out_yaw.resize(T);

  int idx = ref_->nearestIndex(opp_x, opp_y);
  int N = static_cast<int>(ref_->size());

  const auto& rx = ref_->x();
  const auto& ry = ref_->y();
  const auto& ryaw = ref_->yaw();

  double dist_per_step = opp_v * dt;
  double accumulated = 0.0;

  for (int t = 0; t < T; ++t) {
    out_x[t] = rx[idx];
    out_y[t] = ry[idx];
    out_yaw[t] = ryaw[idx];

    accumulated += dist_per_step;
    while (accumulated > 0) {
      int next_idx = (idx + 1) % N;
      double seg_len = std::hypot(rx[next_idx] - rx[idx], ry[next_idx] - ry[idx]);
      if (seg_len < 1e-9) {
        idx = next_idx;
        continue;
      }
      if (accumulated < seg_len) {
        break;
      }
      accumulated -= seg_len;
      idx = next_idx;
    }
  }
}

// -----------------------------
// LocalPathPlanner implementation
// -----------------------------
LocalPathPlanner::LocalPathPlanner(const std::string& map_dir,
                                   const std::string& map_yaml,
                                   const std::string& waypoints_csv,
                                   const LocalPlannerConfig& config)
    : config_(config),
      checker_(map_dir, map_yaml) {
  ref_ = std::make_shared<ReferenceLine>(waypoints_csv, true);
  ref_->load();

  opp_pred_ = std::make_shared<OpponentPredictor>(ref_);
  T_ = static_cast<int>(config_.horizon_s / config_.dt);
}

double LocalPathPlanner::wrapPi(double angle) {
  return std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
}

std::array<std::array<double, 2>, 4> LocalPathPlanner::obbCorners(
    double x, double y, double yaw, double L, double W) {
  double hl = 0.5 * L;
  double hw = 0.5 * W;
  double c = std::cos(yaw);
  double s = std::sin(yaw);

  std::array<std::array<double, 2>, 4> corners;

  // Local corners: [hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]
  std::array<std::array<double, 2>, 4> local = {{
      {hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}
  }};

  for (int i = 0; i < 4; ++i) {
    corners[i][0] = x + local[i][0] * c - local[i][1] * s;
    corners[i][1] = y + local[i][0] * s + local[i][1] * c;
  }

  return corners;
}

bool LocalPathPlanner::obbIntersect(
    const std::array<std::array<double, 2>, 4>& c1,
    const std::array<std::array<double, 2>, 4>& c2) {
  // SAT algorithm
  auto getAxes = [](const std::array<std::array<double, 2>, 4>& c) {
    std::array<std::array<double, 2>, 2> axes;
    for (int i = 0; i < 2; ++i) {
      double ex = c[(i + 1) % 4][0] - c[i][0];
      double ey = c[(i + 1) % 4][1] - c[i][1];
      double len = std::hypot(ex, ey) + 1e-9;
      axes[i][0] = -ey / len;
      axes[i][1] = ex / len;
    }
    return axes;
  };

  auto project = [](const std::array<std::array<double, 2>, 4>& pts,
                    const std::array<double, 2>& axis) {
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();
    for (int i = 0; i < 4; ++i) {
      double proj = pts[i][0] * axis[0] + pts[i][1] * axis[1];
      min_val = std::min(min_val, proj);
      max_val = std::max(max_val, proj);
    }
    return std::make_pair(min_val, max_val);
  };

  auto axes1 = getAxes(c1);
  auto axes2 = getAxes(c2);

  for (const auto& axis : axes1) {
    auto [a_min, a_max] = project(c1, axis);
    auto [b_min, b_max] = project(c2, axis);
    if (a_max < b_min || b_max < a_min) return false;
  }

  for (const auto& axis : axes2) {
    auto [a_min, a_max] = project(c1, axis);
    auto [b_min, b_max] = project(c2, axis);
    if (a_max < b_min || b_max < a_min) return false;
  }

  return true;
}

std::vector<TrajectoryPoint> LocalPathPlanner::rolloutFollowPath(
    double x0, double y0, double yaw0, double v0,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    const std::vector<double>& v_des) const {

  std::vector<TrajectoryPoint> traj(T_);
  double x = x0, y = y0, yaw = yaw0, v = v0;

  const double wb = config_.vehicle.wheelbase;
  const double max_steer = config_.vehicle.max_steer;
  const double max_accel = config_.vehicle.max_accel;
  const double max_decel = config_.vehicle.max_decel;
  const double dt = config_.dt;

  size_t M = path_x.size();

  // Simple nearest point lookup helper
  auto findNearest = [&](double px, double py) -> size_t {
    double min_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    for (size_t i = 0; i < M; ++i) {
      double dx = path_x[i] - px;
      double dy = path_y[i] - py;
      double d = dx * dx + dy * dy;
      if (d < min_dist) {
        min_dist = d;
        min_idx = i;
      }
    }
    return min_idx;
  };

  // Lookahead target helper (with extrapolation at path end)
  auto lookaheadTarget = [&](size_t idx0, double Ld) -> std::pair<double, double> {
    double acc = 0.0;
    size_t i = idx0;
    while (acc < Ld && i + 1 < M) {
      double dx = path_x[i + 1] - path_x[i];
      double dy = path_y[i + 1] - path_y[i];
      acc += std::hypot(dx, dy);
      ++i;
    }

    // If reached end of path, extrapolate in the direction of last segment
    if (i >= M - 1 && M >= 2) {
      double last_dx = path_x[M - 1] - path_x[M - 2];
      double last_dy = path_y[M - 1] - path_y[M - 2];
      double seg_len = std::hypot(last_dx, last_dy);
      if (seg_len > 1e-6) {
        double remaining = Ld - acc;
        if (remaining > 0) {
          double ext_x = path_x[M - 1] + (last_dx / seg_len) * remaining;
          double ext_y = path_y[M - 1] + (last_dy / seg_len) * remaining;
          return {ext_x, ext_y};
        }
      }
      return {path_x[M - 1], path_y[M - 1]};
    }

    i = std::min(i, M - 1);
    return {path_x[i], path_y[i]};
  };

  for (int t = 0; t < T_; ++t) {
    traj[t].x = x;
    traj[t].y = y;
    traj[t].yaw = yaw;
    traj[t].v = v;

    size_t idx = findNearest(x, y);

    double Ld = std::clamp(3.0 + 0.35 * v, 3.0, 12.0);
    auto [target_x, target_y] = lookaheadTarget(idx, Ld);

    // Pure pursuit steering
    double dx = target_x - x;
    double dy = target_y - y;
    double alpha = wrapPi(std::atan2(dy, dx) - yaw);
    double steer = std::atan2(2.0 * wb * std::sin(alpha), Ld);
    steer = std::clamp(steer, -max_steer, max_steer);

    // Longitudinal control
    double vd = v_des[std::min(idx, M - 1)];
    double a = std::clamp((vd - v) * 1.6, -max_decel, max_accel);

    // Kinematic bicycle update
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw = wrapPi(yaw + (v / wb) * std::tan(steer) * dt);
    v = std::max(0.0, v + a * dt);
  }

  return traj;
}

std::vector<double> LocalPathPlanner::computeCurvature(
    const std::vector<double>& x, const std::vector<double>& y) {
  return computePolylineCurvature(x, y);
}

double LocalPathPlanner::sDeltaLoop(double s_from, double s_to) const {
  double d = s_to - s_from;
  double half = 0.5 * ref_->trackLength();
  if (d < -half) d += ref_->trackLength();
  else if (d > half) d -= ref_->trackLength();
  return d;
}

double LocalPathPlanner::computeReward(
    const std::vector<TrajectoryPoint>& traj,
    const std::vector<double>& cand_kappa,
    double lat_offset,
    const std::vector<double>& opp_x,
    const std::vector<double>& opp_y,
    const std::vector<double>& opp_yaw) const {

  int T = static_cast<int>(traj.size());
  const auto& W = config_.weights;

  // Progress reward
  double s0 = ref_->nearestS(traj[0].x, traj[0].y);
  double s1 = ref_->nearestS(traj[T - 1].x, traj[T - 1].y);
  double prog_s = sDeltaLoop(s0, s1);
  double R = W.w_progress * std::max(0.0, prog_s);

  // Speed reward
  double sum_v = 0.0;
  for (int t = 0; t < T; ++t) sum_v += traj[t].v;
  R += W.w_speed * (sum_v / T);

  // Smoothness penalty
  double sum_dv = 0.0;
  for (int t = 1; t < T; ++t) {
    sum_dv += std::abs(traj[t].v - traj[t - 1].v);
  }
  R -= W.w_smooth * (sum_dv / T);

  // Curvature penalty
  double sum_kappa = 0.0;
  for (double k : cand_kappa) sum_kappa += std::abs(k);
  R -= W.w_curv * (sum_kappa / cand_kappa.size());

  // Lateral offset penalty
  R -= W.w_far_from_ref * std::abs(lat_offset);

  // Offroad penalty
  int off_cnt = 0;
  for (int t = 0; t < T; t += config_.offroad_check_stride) {
    if (!checker_.isDrivable(traj[t].x, traj[t].y)) {
      ++off_cnt;
    }
  }
  if (off_cnt > 0) {
    R -= W.w_offroad * off_cnt;
  }

  // Collision penalty (OBB)
  int collision_cnt = 0;
  double min_center_dist = std::numeric_limits<double>::max();
  int T_opp = std::min(T, static_cast<int>(opp_x.size()));

  for (int t = 0; t < T_opp; ++t) {
    double ex = traj[t].x;
    double ey = traj[t].y;
    double eyaw = traj[t].yaw;
    double ox = opp_x[t];
    double oy = opp_y[t];
    double oyaw = opp_yaw[t];

    double dist = std::hypot(ex - ox, ey - oy);
    min_center_dist = std::min(min_center_dist, dist);

    // Early skip if too far
    if (dist > (config_.ego_length + config_.opp_length)) continue;

    auto ego_box = obbCorners(ex, ey, eyaw, config_.ego_length, config_.ego_width);
    auto opp_box = obbCorners(ox, oy, oyaw, config_.opp_length, config_.opp_width);
    if (obbIntersect(ego_box, opp_box)) {
      ++collision_cnt;
    }
  }

  if (collision_cnt > 0) {
    R -= W.w_collision * collision_cnt;
  }

  // Overtake reward
  double s_ego_end = ref_->nearestS(traj[T - 1].x, traj[T - 1].y);
  int opp_end_idx = std::min(T - 1, static_cast<int>(opp_x.size()) - 1);
  double s_opp_end = ref_->nearestS(opp_x[opp_end_idx], opp_y[opp_end_idx]);
  double ds_end = sDeltaLoop(s_opp_end, s_ego_end);

  R += W.w_overtake * std::clamp(ds_end, -5.0, 20.0);

  // Bonus for completing overtake
  if (ds_end > config_.overtake_margin_s) {
    R += W.w_overtake * 2.0 * std::min(ds_end - config_.overtake_margin_s, 10.0);
  }

  // Near-pass bonus
  if (min_center_dist < config_.near_pass_bonus_dist && collision_cnt == 0) {
    R += 5.0 * (config_.near_pass_bonus_dist - min_center_dist);
  }

  return R;
}

PlanResult LocalPathPlanner::plan(
    double ego_x, double ego_y, double ego_yaw, double ego_v,
    double opp_x, double opp_y, double opp_v) {

  // Get base segment ahead
  int idx0 = ref_->nearestIndex(ego_x, ego_y);
  std::vector<int> idxs = ref_->getSegmentIndices(idx0, config_.path_ahead_m);

  // Extract base data for indices
  std::vector<double> base_v(idxs.size());
  std::vector<double> base_kappa(idxs.size());
  for (size_t i = 0; i < idxs.size(); ++i) {
    base_v[i] = ref_->v_ref()[idxs[i]];
    base_kappa[i] = ref_->kappa()[idxs[i]];
  }

  // Pre-compute v_cap based on curvature
  std::vector<double> v_cap(idxs.size());
  for (size_t i = 0; i < idxs.size(); ++i) {
    double abs_k = std::abs(base_kappa[i]);
    if (abs_k > 1e-6) {
      v_cap[i] = std::min(std::sqrt(config_.vehicle.max_lat_accel / abs_k), 60.0);
    } else {
      v_cap[i] = 60.0;
    }
  }

  // Predict opponent trajectory
  std::vector<double> opp_future_x, opp_future_y, opp_future_yaw;
  opp_pred_->predictStates(opp_x, opp_y, opp_v,
                           config_.horizon_s, config_.dt,
                           opp_future_x, opp_future_y, opp_future_yaw);

  PlanResult best;
  best.reward = -std::numeric_limits<double>::max();

  std::vector<PlanResult::DebugCandidate> debug_results;

  for (double lat : config_.lateral_offsets) {
    // Generate candidate path
    std::vector<double> cand_x, cand_y;
    ref_->getShiftedLine(idxs, lat, cand_x, cand_y);

    // Compute curvature for candidate
    std::vector<double> cand_kappa = computeCurvature(cand_x, cand_y);

    for (double sc : config_.speed_scales) {
      // Compute desired velocity
      std::vector<double> v_des(idxs.size());
      for (size_t i = 0; i < idxs.size(); ++i) {
        v_des[i] = std::min(base_v[i] * sc, v_cap[i]);
      }

      // Rollout trajectory
      std::vector<TrajectoryPoint> traj = rolloutFollowPath(
          ego_x, ego_y, ego_yaw, ego_v, cand_x, cand_y, v_des);

      // Compute reward
      double R = computeReward(traj, cand_kappa, lat,
                               opp_future_x, opp_future_y, opp_future_yaw);

      // Apply hysteresis: penalize changing lateral offset direction
      if (has_prev_offset_) {
        double offset_change = std::abs(lat - prev_lat_offset_);
        R -= config_.lat_change_penalty * offset_change;

        // Extra penalty for crossing center (changing sides)
        if ((lat > 0.0 && prev_lat_offset_ < 0.0) || (lat < 0.0 && prev_lat_offset_ > 0.0)) {
          R -= config_.lat_change_penalty * 2.0;  // Additional penalty for side change
        }
      }

      debug_results.push_back({R, lat, sc});

      if (R > best.reward) {
        best.reward = R;
        best.lat_offset = lat;
        best.speed_scale = sc;
        best.trajectory = std::move(traj);
        best.candidate_path.clear();
        for (size_t i = 0; i < cand_x.size(); ++i) {
          best.candidate_path.emplace_back(cand_x[i], cand_y[i]);
        }
        best.v_desired = std::move(v_des);
      }
    }
  }

  // Sort debug results and keep top 12
  std::sort(debug_results.begin(), debug_results.end(),
            [](const auto& a, const auto& b) { return a.reward > b.reward; });

  best.debug_top.clear();
  for (size_t i = 0; i < std::min(debug_results.size(), size_t(12)); ++i) {
    best.debug_top.push_back(debug_results[i]);
  }

  // Update hysteresis state
  prev_lat_offset_ = best.lat_offset;
  has_prev_offset_ = true;

  return best;
}

}  // namespace scenario_director
