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

    // Compute tangent direction from adjacent points (more robust than stored yaw)
    int idx_prev = (idx == 0) ? (loop_ ? static_cast<int>(N_) - 1 : 0) : idx - 1;
    int idx_next = (idx == static_cast<int>(N_) - 1) ? (loop_ ? 0 : idx) : idx + 1;

    double dx = x_[idx_next] - x_[idx_prev];
    double dy = y_[idx_next] - y_[idx_prev];
    double len = std::hypot(dx, dy);

    double nx, ny;
    if (len > 1e-6) {
      // Normal is perpendicular to tangent (rotate 90 degrees CCW)
      nx = -dy / len;
      ny = dx / len;
    } else {
      // Fallback to stored yaw if points are too close
      nx = -std::sin(yaw_[idx]);
      ny = std::cos(yaw_[idx]);
    }

    out_x[i] = x_[idx] + offset_m * nx;
    out_y[i] = y_[idx] + offset_m * ny;
  }
}

void ReferenceLine::getLateBrakingLine(const std::vector<int>& indices,
                                        size_t brake_point_idx, size_t apex_idx,
                                        int corner_sign,
                                        double outside_offset, double inside_offset,
                                        double sharpness, double exit_distance,
                                        std::vector<double>& out_x, std::vector<double>& out_y) const {
  size_t M = indices.size();
  out_x.resize(M);
  out_y.resize(M);

  // 안전 검증
  brake_point_idx = std::min(brake_point_idx, M - 1);
  apex_idx = std::min(apex_idx, M - 1);
  if (brake_point_idx >= apex_idx) {
    brake_point_idx = (apex_idx > 0) ? apex_idx - 1 : 0;
  }

  // 코너 방향에 따른 오프셋 부호 결정
  // corner_sign > 0: 우회전 -> 인사이드는 오른쪽(+), 아웃사이드는 왼쪽(-)
  // corner_sign < 0: 좌회전 -> 인사이드는 왼쪽(-), 아웃사이드는 오른쪽(+)
  const double actual_outside = -corner_sign * outside_offset;  // 바깥쪽
  const double actual_inside = corner_sign * inside_offset;     // 안쪽

  // apex 후 복귀 구간 계산
  size_t exit_end_idx = apex_idx;
  double accumulated_dist = 0.0;
  for (size_t i = apex_idx; i + 1 < M && accumulated_dist < exit_distance; ++i) {
    int idx_curr = indices[i];
    int idx_next = indices[i + 1];
    accumulated_dist += std::hypot(x_[idx_next] - x_[idx_curr], y_[idx_next] - y_[idx_curr]);
    exit_end_idx = i + 1;
  }

  for (size_t i = 0; i < M; ++i) {
    int idx = indices[i];

    // 법선 벡터 계산
    int idx_prev = (idx == 0) ? (loop_ ? static_cast<int>(N_) - 1 : 0) : idx - 1;
    int idx_next_pt = (idx == static_cast<int>(N_) - 1) ? (loop_ ? 0 : idx) : idx + 1;

    double dx = x_[idx_next_pt] - x_[idx_prev];
    double dy = y_[idx_next_pt] - y_[idx_prev];
    double len = std::hypot(dx, dy);

    double nx, ny;
    if (len > 1e-6) {
      nx = -dy / len;
      ny = dx / len;
    } else {
      nx = -std::sin(yaw_[idx]);
      ny = std::cos(yaw_[idx]);
    }

    // 위치에 따른 오프셋 계산
    double offset;

    if (i <= brake_point_idx) {
      // 브레이킹 포인트 전: 바깥쪽 유지
      offset = actual_outside;
    } else if (i <= apex_idx) {
      // 브레이킹 ~ apex: 급격한 전환 (out -> in)
      double t = static_cast<double>(i - brake_point_idx) /
                 static_cast<double>(apex_idx - brake_point_idx);
      // sharpness가 높을수록 나중에 급격히 꺾임
      // t^sharpness 곡선 사용
      double curved_t = std::pow(t, sharpness);
      offset = actual_outside + (actual_inside - actual_outside) * curved_t;
    } else if (i <= exit_end_idx) {
      // apex ~ exit: 레이싱라인으로 복귀 (in -> 0)
      double t = static_cast<double>(i - apex_idx) /
                 static_cast<double>(exit_end_idx - apex_idx);
      // 부드러운 복귀 (1 - (1-t)^2)
      double smooth_t = 1.0 - (1.0 - t) * (1.0 - t);
      offset = actual_inside * (1.0 - smooth_t);
    } else {
      // exit 이후: 레이싱라인
      offset = 0.0;
    }

    out_x[i] = x_[idx] + offset * nx;
    out_y[i] = y_[idx] + offset * ny;
  }
}

// -----------------------------
// OpponentPredictor implementation
// -----------------------------
OpponentPredictor::OpponentPredictor(std::shared_ptr<ReferenceLine> ref)
    : ref_(std::move(ref)) {}

double OpponentPredictor::estimateSpeedRatio(double opp_x, double opp_y, double opp_v) const {
  int idx = ref_->nearestIndex(opp_x, opp_y);
  double v_ref = ref_->v_ref()[idx];

  if (v_ref < 1.0) {
    // v_ref가 너무 작으면 기본값 사용
    return 1.0;
  }

  // 현재 속도 / 레퍼런스 속도 = speed_ratio
  double ratio = opp_v / v_ref;

  // 합리적인 범위로 클램프 (0.3 ~ 1.5)
  return std::clamp(ratio, 0.3, 1.5);
}

void OpponentPredictor::predictStates(double opp_x, double opp_y, double opp_v,
                                       double horizon_s, double dt,
                                       std::vector<double>& out_x,
                                       std::vector<double>& out_y,
                                       std::vector<double>& out_yaw) const {
  // 기존 등속 방식 (하위 호환성)
  std::vector<double> out_v;
  predictStatesWithVRef(opp_x, opp_y, opp_v, 0.0, horizon_s, dt,
                        out_x, out_y, out_yaw, out_v);
}

void OpponentPredictor::predictStatesWithVRef(double opp_x, double opp_y, double opp_v,
                                               double speed_ratio,
                                               double horizon_s, double dt,
                                               std::vector<double>& out_x,
                                               std::vector<double>& out_y,
                                               std::vector<double>& out_yaw,
                                               std::vector<double>& out_v) const {
  int T = static_cast<int>(horizon_s / dt);
  out_x.resize(T);
  out_y.resize(T);
  out_yaw.resize(T);
  out_v.resize(T);

  int idx = ref_->nearestIndex(opp_x, opp_y);
  int N = static_cast<int>(ref_->size());

  const auto& rx = ref_->x();
  const auto& ry = ref_->y();
  const auto& ryaw = ref_->yaw();
  const auto& rv = ref_->v_ref();

  // speed_ratio가 0이면 현재 속도에서 추정
  double actual_ratio = speed_ratio;
  if (actual_ratio <= 0.0) {
    actual_ratio = estimateSpeedRatio(opp_x, opp_y, opp_v);
  }

  double accumulated = 0.0;

  for (int t = 0; t < T; ++t) {
    out_x[t] = rx[idx];
    out_y[t] = ry[idx];
    out_yaw[t] = ryaw[idx];

    // 핵심: v_ref × speed_ratio로 해당 위치에서의 속도 예측
    out_v[t] = rv[idx] * actual_ratio;

    // 해당 속도로 이동
    double dist_per_step = out_v[t] * dt;
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

LocalPathPlanner::PassabilityInfo LocalPathPlanner::checkPassability(
    double opp_x, double opp_y, double opp_yaw) const {

  PassabilityInfo info;

  // 상대 차량 기준 좌/우 방향 벡터
  const double left_nx = -std::sin(opp_yaw);
  const double left_ny = std::cos(opp_yaw);
  const double right_nx = std::sin(opp_yaw);
  const double right_ny = -std::cos(opp_yaw);

  // 상대 차량 폭의 절반
  const double half_opp_width = config_.opp_width * 0.5;

  // 왼쪽 갭 측정: 상대 차량 왼쪽 끝에서 벽까지 거리
  double left_edge_x = opp_x + left_nx * half_opp_width;
  double left_edge_y = opp_y + left_ny * half_opp_width;

  // 오른쪽 갭 측정: 상대 차량 오른쪽 끝에서 벽까지 거리
  double right_edge_x = opp_x + right_nx * half_opp_width;
  double right_edge_y = opp_y + right_ny * half_opp_width;

  // 벽까지 거리 측정 (최대 10m까지 체크)
  constexpr double MAX_CHECK_DIST = 10.0;
  constexpr double STEP = 0.3;

  // 왼쪽 갭 측정
  info.left_gap = 0.0;
  for (double d = STEP; d <= MAX_CHECK_DIST; d += STEP) {
    double check_x = left_edge_x + left_nx * d;
    double check_y = left_edge_y + left_ny * d;
    if (!checker_.isDrivable(check_x, check_y)) {
      info.left_gap = d - STEP;
      break;
    }
    info.left_gap = d;
  }

  // 오른쪽 갭 측정
  info.right_gap = 0.0;
  for (double d = STEP; d <= MAX_CHECK_DIST; d += STEP) {
    double check_x = right_edge_x + right_nx * d;
    double check_y = right_edge_y + right_ny * d;
    if (!checker_.isDrivable(check_x, check_y)) {
      info.right_gap = d - STEP;
      break;
    }
    info.right_gap = d;
  }

  // 통과 가능 여부 판단: 갭 > ego 차량 폭 + 안전 마진
  const double required_gap = config_.ego_width + config_.pass_gap_margin;
  info.left_passable = (info.left_gap >= required_gap);
  info.right_passable = (info.right_gap >= required_gap);

  return info;
}

double LocalPathPlanner::computeBrakingDistance(double current_speed, double apex_curvature,
                                                 double delay_factor) const {
  const auto& lb_cfg = config_.strategy.overtake.late_braking;
  const double max_decel = (lb_cfg.max_decel_override > 0.0)
                             ? lb_cfg.max_decel_override
                             : config_.vehicle.max_decel;

  // apex에서 필요한 속도: v_apex = sqrt(max_lat_accel / curvature)
  const double abs_curvature = std::max(std::abs(apex_curvature), 1e-6);
  const double v_apex = std::sqrt(config_.vehicle.max_lat_accel / abs_curvature);

  // 현재 속도에서 apex 속도까지 감속에 필요한 거리
  // d = (v0^2 - v_apex^2) / (2 * decel)
  const double v0_sq = current_speed * current_speed;
  const double v_apex_sq = v_apex * v_apex;

  if (v0_sq <= v_apex_sq) {
    // 이미 충분히 느림, 브레이킹 불필요
    return 0.0;
  }

  const double base_distance = (v0_sq - v_apex_sq) / (2.0 * max_decel);

  // 안전 마진 적용
  const double safe_distance = base_distance * lb_cfg.safety_margin;

  // delay_factor 적용: 브레이킹 시작을 늦춤
  // delay_factor = 0.0 -> 정상 브레이킹
  // delay_factor = 0.5 -> 50% 늦게 브레이킹 (거리 50% 줄임)
  const double delayed_distance = safe_distance * (1.0 - delay_factor);

  return delayed_distance;
}

std::vector<LocalPathPlanner::LateBrakingCandidate> LocalPathPlanner::generateLateBrakingCandidates(
    const std::vector<int>& indices,
    const std::vector<double>& base_v,
    const std::vector<double>& base_kappa,
    const std::vector<double>& ds_from_start,
    double current_speed,
    size_t apex_idx, double apex_curvature, int corner_sign) const {

  std::vector<LateBrakingCandidate> candidates;
  const auto& lb_cfg = config_.strategy.overtake.late_braking;

  if (!lb_cfg.enabled || std::abs(apex_curvature) < lb_cfg.min_corner_curvature) {
    return candidates;  // late braking 비활성화 또는 코너가 너무 완만
  }

  // 다양한 delay factor로 후보 생성 (0.1 ~ delay_factor까지)
  std::vector<double> delay_factors = {0.0, lb_cfg.delay_factor * 0.5, lb_cfg.delay_factor};

  // 다양한 inside offset으로 후보 생성
  std::vector<double> inside_offsets = {
    lb_cfg.inside_offset * 0.5,
    lb_cfg.inside_offset,
    lb_cfg.inside_offset * 1.5
  };

  const double apex_distance = ds_from_start[apex_idx];

  for (double delay : delay_factors) {
    // 브레이킹 거리 계산
    double brake_distance = computeBrakingDistance(current_speed, apex_curvature, delay);

    // 브레이킹 포인트 = apex 위치 - 브레이킹 거리
    double brake_point_s = apex_distance - brake_distance;
    if (brake_point_s < 0.0) {
      brake_point_s = 0.0;  // 이미 브레이킹 구간에 있음
    }

    // brake_point_s에 해당하는 인덱스 찾기
    size_t brake_idx = 0;
    for (size_t i = 0; i < indices.size(); ++i) {
      if (ds_from_start[i] >= brake_point_s) {
        brake_idx = i;
        break;
      }
    }

    for (double inside_off : inside_offsets) {
      LateBrakingCandidate cand;
      cand.brake_point_idx = brake_idx;
      cand.apex_idx = apex_idx;
      cand.corner_sign = corner_sign;
      cand.outside_offset = lb_cfg.outside_offset;
      cand.inside_offset = inside_off;

      // 경로 생성
      ref_->getLateBrakingLine(indices, brake_idx, apex_idx, corner_sign,
                               lb_cfg.outside_offset, inside_off,
                               lb_cfg.transition_sharpness, lb_cfg.exit_recovery_distance,
                               cand.path_x, cand.path_y);

      // 속도 프로파일 생성
      cand.v_desired.resize(indices.size());
      for (size_t i = 0; i < indices.size(); ++i) {
        double v = base_v[i];

        if (i >= brake_idx && i <= apex_idx) {
          // 브레이킹 구간: 점진적 감속
          double t = static_cast<double>(i - brake_idx) /
                     static_cast<double>(apex_idx - brake_idx + 1);
          double abs_k = std::max(std::abs(apex_curvature), 1e-6);
          double v_apex = std::sqrt(config_.vehicle.max_lat_accel / abs_k);
          v = current_speed + (v_apex - current_speed) * t;
        } else if (i > apex_idx) {
          // 탈출 구간: 가속
          double exit_boost = config_.strategy.overtake.outside_exit_boost;
          v *= exit_boost;
        }

        // 곡률 기반 속도 제한
        double abs_k = std::abs(base_kappa[i]);
        if (abs_k > 1e-6) {
          double v_cap = std::sqrt(config_.vehicle.max_lat_accel / abs_k);
          v = std::min(v, v_cap);
        }

        cand.v_desired[i] = std::max(v, 5.0);  // 최소 속도 보장
      }

      candidates.push_back(std::move(cand));
    }
  }

  return candidates;
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
    double opp_x, double opp_y, double opp_v,
    bool overtake_flag, int overtake_phase) {

  // Overtake phases: 0=NONE, 1=APPROACH, 2=POSITION, 3=EXECUTE, 4=COMPLETE
  const bool phase_approach = (overtake_phase == 1);
  const bool phase_position = (overtake_phase == 2);
  const bool phase_execute = (overtake_phase == 3);
  const bool phase_complete = (overtake_phase == 4);

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

  // Distance along track from current position for each index
  std::vector<double> ds_from_start(idxs.size(), 0.0);
  const double s0 = ref_->s()[idx0];
  for (size_t i = 0; i < idxs.size(); ++i) {
    ds_from_start[i] = sDeltaLoop(s0, ref_->s()[idxs[i]]);
  }

  auto findApex = [&](double search_dist, double curv_thresh,
                      size_t& out_idx, double& out_kappa) -> bool {
    if (search_dist <= 0.0) {
      return false;
    }
    double best_abs = curv_thresh;
    bool found = false;
    for (size_t i = 0; i < idxs.size(); ++i) {
      if (ds_from_start[i] > search_dist) {
        break;
      }
      double abs_k = std::abs(base_kappa[i]);
      if (abs_k > best_abs) {
        best_abs = abs_k;
        out_idx = i;
        out_kappa = base_kappa[i];
        found = true;
      }
    }
    return found;
  };

  size_t slow_apex_i = 0;
  double slow_apex_k = 0.0;
  const bool has_slow_apex =
      config_.strategy.slow_in_out.enabled &&
      findApex(config_.strategy.slow_in_out.apex_search_distance,
               config_.strategy.slow_in_out.curvature_threshold,
               slow_apex_i, slow_apex_k);
  (void)slow_apex_k;

  size_t overtake_apex_i = 0;
  double overtake_apex_k = 0.0;
  const bool has_overtake_apex =
      findApex(config_.strategy.overtake.apex_search_distance,
               config_.strategy.overtake.min_corner_curvature,
               overtake_apex_i, overtake_apex_k);

  std::vector<double> dist_to_slow_apex(idxs.size(), 0.0);
  std::vector<double> dist_from_slow_apex(idxs.size(), 0.0);
  if (has_slow_apex) {
    const double apex_s = ref_->s()[idxs[slow_apex_i]];
    for (size_t i = 0; i < idxs.size(); ++i) {
      dist_to_slow_apex[i] = sDeltaLoop(ref_->s()[idxs[i]], apex_s);
      dist_from_slow_apex[i] = sDeltaLoop(apex_s, ref_->s()[idxs[i]]);
    }
  }

  std::vector<double> dist_to_overtake_apex(idxs.size(), 0.0);
  std::vector<double> dist_from_overtake_apex(idxs.size(), 0.0);
  if (has_overtake_apex) {
    const double apex_s = ref_->s()[idxs[overtake_apex_i]];
    for (size_t i = 0; i < idxs.size(); ++i) {
      dist_to_overtake_apex[i] = sDeltaLoop(ref_->s()[idxs[i]], apex_s);
      dist_from_overtake_apex[i] = sDeltaLoop(apex_s, ref_->s()[idxs[i]]);
    }
  }

  // Opponent context
  const double dx_opp = opp_x - ego_x;
  const double dy_opp = opp_y - ego_y;
  const double opp_dist = std::hypot(dx_opp, dy_opp);
  const double heading_x = std::cos(ego_yaw);
  const double heading_y = std::sin(ego_yaw);
  const double forward_to_opp = dx_opp * heading_x + dy_opp * heading_y;
  const double lateral_to_opp = -dx_opp * heading_y + dy_opp * heading_x;  // 양수 = 상대가 오른쪽
  const bool opp_ahead = forward_to_opp > 0.0;
  const bool opp_beside = std::abs(forward_to_opp) < 5.0 && std::abs(lateral_to_opp) < 4.0;
  const double dummy_trigger_dist = std::max(0.0, config_.strategy.overtake.dummy_trigger_distance);
  const bool dummy_trigger = opp_ahead && (opp_dist <= dummy_trigger_dist);
  // Use overtake_flag from scenario_director for unified overtake state management
  const bool overtake_active = overtake_flag;

  // Slipstream detection (직선에서 상대 바로 뒤에 있을 때)
  const bool in_slipstream = opp_ahead &&
                             forward_to_opp <= config_.strategy.overtake.slipstream_distance &&
                             std::abs(lateral_to_opp) < 2.0;

  // 추월 방향 결정 (상대의 반대편으로)
  const int overtake_side = (lateral_to_opp > 0.0) ? -1 : 1;  // 상대가 오른쪽이면 왼쪽으로 추월

  // 상대 차량과 벽 사이 갭 체크 (전방에 있고 가까울 때만)
  PassabilityInfo passability;
  const bool check_gap = opp_ahead && forward_to_opp < config_.gap_check_distance;
  if (check_gap) {
    // 상대 차량의 yaw 추정 (레퍼런스 라인 기준)
    int opp_idx = ref_->nearestIndex(opp_x, opp_y);
    double opp_yaw_est = ref_->yaw()[opp_idx];
    passability = checkPassability(opp_x, opp_y, opp_yaw_est);
  }

  // Dummy (feint) state update
  if (!dummy_trigger) {
    dummy_active_ = false;
    dummy_steps_total_ = 0;
    dummy_steps_elapsed_ = 0;
    dummy_has_side_ = false;
  } else {
    if (!dummy_active_) {
      dummy_active_ = true;
      const double dt = std::max(1e-3, config_.dt);
      dummy_steps_total_ = std::max(1, static_cast<int>(
          std::round(config_.strategy.overtake.dummy_duration / dt)));
      dummy_steps_elapsed_ = 0;
      if (has_prev_offset_ && std::abs(prev_lat_offset_) > 1e-3) {
        dummy_side_ = (prev_lat_offset_ > 0.0) ? 1 : -1;
      } else if (has_overtake_apex) {
        dummy_side_ = (overtake_apex_k >= 0.0) ? 1 : -1;
      } else {
        dummy_side_ = 1;
      }
      dummy_has_side_ = true;
    } else if (dummy_steps_total_ > 0) {
      dummy_steps_elapsed_ = std::min(dummy_steps_elapsed_ + 1, dummy_steps_total_);
    }
  }

  const double dummy_switch_th =
      std::clamp(config_.strategy.overtake.dummy_switch_threshold, 0.0, 1.0);
  const double dummy_progress =
      (dummy_steps_total_ > 0) ? static_cast<double>(dummy_steps_elapsed_) / dummy_steps_total_ : 0.0;
  const int dummy_pref_sign =
      (dummy_active_ && dummy_has_side_ && dummy_progress < dummy_switch_th) ? dummy_side_ :
      (dummy_active_ && dummy_has_side_ ? -dummy_side_ : 0);

  // Slow in/out speed scale profile
  std::vector<double> slow_scale(idxs.size(), 1.0);
  if (has_slow_apex) {
    double entry_scale = config_.strategy.slow_in_out.entry_speed_scale;
    double exit_scale = config_.strategy.slow_in_out.exit_speed_scale;
    if (overtake_active) {
      entry_scale = config_.strategy.slow_in_out.overtake_entry_speed_scale;
      exit_scale = config_.strategy.slow_in_out.overtake_exit_speed_scale;
    }
    if (!config_.strategy.slow_in_out.aggressive_exit) {
      exit_scale = std::min(exit_scale, 1.0);
    }
    for (size_t i = 0; i < idxs.size(); ++i) {
      if (dist_to_slow_apex[i] >= 0.0 &&
          dist_to_slow_apex[i] <= config_.strategy.slow_in_out.entry_distance) {
        slow_scale[i] *= entry_scale;
      }
      if (dist_from_slow_apex[i] >= 0.0 &&
          dist_from_slow_apex[i] <= config_.strategy.slow_in_out.exit_distance) {
        slow_scale[i] *= exit_scale;
      }
    }
  }

  // Chicane detection for outside strategy
  bool chicane_detected = false;
  if (config_.strategy.overtake.chicane_detection_dist > 0.0) {
    bool has_pos = false;
    bool has_neg = false;
    const double min_k = config_.strategy.overtake.min_corner_curvature;
    for (size_t i = 0; i < idxs.size(); ++i) {
      if (ds_from_start[i] > config_.strategy.overtake.chicane_detection_dist) {
        break;
      }
      if (base_kappa[i] > min_k) has_pos = true;
      if (base_kappa[i] < -min_k) has_neg = true;
      if (has_pos && has_neg) {
        chicane_detected = true;
        break;
      }
    }
  }

  const int corner_sign =
      has_overtake_apex ? ((overtake_apex_k > 1e-6) ? 1 : (overtake_apex_k < -1e-6 ? -1 : 0)) : 0;
  const int inside_sign = corner_sign;
  const int outside_sign = -corner_sign;

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

  // Predict opponent trajectory (v_ref 기반 예측)
  // 현재 속도에서 speed_ratio 추정 후, 각 위치의 v_ref × ratio로 예측
  std::vector<double> opp_future_x, opp_future_y, opp_future_yaw, opp_future_v;
  const double estimated_ratio = opp_pred_->estimateSpeedRatio(opp_x, opp_y, opp_v);
  opp_pred_->predictStatesWithVRef(opp_x, opp_y, opp_v, estimated_ratio,
                                    config_.horizon_s, config_.dt,
                                    opp_future_x, opp_future_y, opp_future_yaw, opp_future_v);

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
        double v = base_v[i] * sc;

        if (has_slow_apex) {
          v *= slow_scale[i];
        }

        // Phase-based speed adjustments
        if (phase_approach && in_slipstream) {
          // 접근 단계 + 슬립스트림: 속도 부스트
          v *= config_.strategy.overtake.slipstream_speed_boost;
        }
        if (phase_position || phase_execute) {
          // 위치 선점/실행 단계: 공격적 속도
          v *= config_.strategy.overtake.execute_speed_boost;
        }

        // Chicane에서 아웃사이드 전략 (late braking이 아닌 경우)
        if (overtake_active && has_overtake_apex && chicane_detected) {
          const bool in_entry_zone =
              dist_to_overtake_apex[i] >= 0.0 &&
              dist_to_overtake_apex[i] <= config_.strategy.slow_in_out.entry_distance;
          const bool in_exit_zone =
              dist_from_overtake_apex[i] >= 0.0 &&
              dist_from_overtake_apex[i] <= config_.strategy.slow_in_out.exit_distance;

          if (outside_sign != 0 && lat * outside_sign > 0.1) {
            if (in_entry_zone) {
              v *= config_.strategy.overtake.outside_entry_speed_scale;
            }
            if (in_exit_zone) {
              v *= config_.strategy.overtake.outside_exit_boost;
            }
          }
        }

        v_des[i] = std::min(v, v_cap[i]);
      }

      // Rollout trajectory
      std::vector<TrajectoryPoint> traj = rolloutFollowPath(
          ego_x, ego_y, ego_yaw, ego_v, cand_x, cand_y, v_des);

      // Compute reward
      double R = computeReward(traj, cand_kappa, lat,
                               opp_future_x, opp_future_y, opp_future_yaw);

      // ============================================
      // 갭 통과 가능성 페널티 (벽과 상대차량 사이)
      // ============================================
      if (check_gap) {
        // lat > 0: 오른쪽으로 가려함, lat < 0: 왼쪽으로 가려함
        // 상대가 전방에 있을 때, 그 옆으로 지나가려면 충분한 갭 필요
        if (lat > 0.5 && !passability.right_passable) {
          // 오른쪽으로 가려는데 오른쪽 갭이 부족
          R -= config_.impassable_penalty;
        }
        if (lat < -0.5 && !passability.left_passable) {
          // 왼쪽으로 가려는데 왼쪽 갭이 부족
          R -= config_.impassable_penalty;
        }

        // 갭이 좁을수록 추가 페널티 (부드러운 감쇠)
        const double required = config_.ego_width + config_.pass_gap_margin;
        if (lat > 0.5) {
          double gap_shortage = std::max(0.0, required - passability.right_gap);
          R -= gap_shortage * 5000.0;  // 갭 부족 1m당 5000 페널티
        }
        if (lat < -0.5) {
          double gap_shortage = std::max(0.0, required - passability.left_gap);
          R -= gap_shortage * 5000.0;
        }
      }

      // Apply hysteresis: penalize changing lateral offset direction
      if (has_prev_offset_) {
        double offset_change = std::abs(lat - prev_lat_offset_);
        R -= config_.lat_change_penalty * offset_change;

        // Extra penalty for crossing center (changing sides)
        if ((lat > 0.0 && prev_lat_offset_ < 0.0) || (lat < 0.0 && prev_lat_offset_ > 0.0)) {
          R -= config_.lat_change_penalty * 2.0;  // Additional penalty for side change
        }
      }

      // Side commitment during overtake: strongly penalize switching sides
      if (overtake_active && committed_side_ != 0) {
        const int lat_sign = (lat > 0.1) ? 1 : ((lat < -0.1) ? -1 : 0);
        if (lat_sign != 0 && lat_sign != committed_side_) {
          // Trying to switch to the opposite side during overtake
          R -= config_.overtake_side_commit_penalty;
        }
      }

      // ============================================
      // Phase-based overtake strategy rewards
      // ============================================
      const double gap_scale = config_.strategy.overtake.gap_reward_scale;
      const double target_offset = config_.strategy.overtake.position_lateral_offset;
      const int lat_sign = (lat > 0.1) ? 1 : ((lat < -0.1) ? -1 : 0);

      if (phase_approach) {
        // 접근 단계: 슬립스트림 위치 유지 (중앙), 추월 방향 준비
        if (in_slipstream && std::abs(lat) < 1.0) {
          R += gap_scale * 0.5;  // 슬립스트림 보너스
        }
        // 추월 방향으로 약간 이동 유도
        if (lat_sign == overtake_side) {
          R += gap_scale * 0.3;
        }
      }

      if (phase_position) {
        // 위치 선점 단계: 상대 옆으로 적극적으로 이동
        const double target_lat = overtake_side * target_offset;
        const double dist_to_target = std::abs(lat - target_lat);
        R -= gap_scale * dist_to_target;  // 목표 위치에 가까울수록 보상

        // 목표 사이드에 있으면 보너스
        if (lat_sign == overtake_side) {
          R += gap_scale * 1.5;
        }
      }

      if (phase_execute) {
        // 실행 단계: 나란히 또는 앞서가기
        const double target_lat = overtake_side * target_offset;
        const double dist_to_target = std::abs(lat - target_lat);
        R -= gap_scale * 0.5 * dist_to_target;

        // 상대 옆에 나란히 있으면 큰 보너스
        if (opp_beside && lat_sign == overtake_side) {
          R += gap_scale * 2.0;
        }

        // 앞으로 나가는 것에 추가 보상
        R += config_.weights.w_overtake * 1.5;
      }

      if (phase_complete) {
        // 완료 단계: 안전하게 앞으로 복귀
        // 레이싱 라인(중앙)으로 부드럽게 복귀 유도
        R -= gap_scale * 0.3 * std::abs(lat);
      }

      // 기존 코너 전략 (코너가 있을 때만)
      if (overtake_active && has_overtake_apex) {
        if (inside_sign != 0 && config_.strategy.overtake.block_pass_offset > 0.0) {
          const double target = inside_sign * config_.strategy.overtake.block_pass_offset;
          R -= config_.weights.w_overtake * 0.5 * std::abs(lat - target);
        }
        if (chicane_detected && outside_sign != 0 && lat * outside_sign > 0.1) {
          R += config_.weights.w_overtake * 1.0;
        }
      }

      // Dummy (feint) strategy
      if (dummy_active_ && dummy_pref_sign != 0) {
        const double dummy_target = dummy_pref_sign * config_.strategy.overtake.dummy_offset;
        R -= config_.weights.w_overtake * 0.3 * std::abs(lat - dummy_target);
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

  // ============================================
  // Late Braking 후보 평가 (브레이킹 포인트 기반)
  // ============================================
  if (overtake_active && has_overtake_apex) {
    // Late braking 후보 생성
    auto lb_candidates = generateLateBrakingCandidates(
        idxs, base_v, base_kappa, ds_from_start,
        ego_v, overtake_apex_i, overtake_apex_k, corner_sign);

    for (const auto& lb_cand : lb_candidates) {
      // Compute curvature for late braking path
      std::vector<double> lb_kappa = computeCurvature(lb_cand.path_x, lb_cand.path_y);

      // Rollout trajectory
      std::vector<TrajectoryPoint> traj = rolloutFollowPath(
          ego_x, ego_y, ego_yaw, ego_v,
          lb_cand.path_x, lb_cand.path_y, lb_cand.v_desired);

      // 평균 lateral offset 계산 (late braking 경로는 가변이므로)
      double avg_lat_offset = 0.0;
      if (!lb_cand.path_x.empty() && !idxs.empty()) {
        // 브레이킹 포인트에서의 offset 사용
        size_t sample_idx = std::min(lb_cand.brake_point_idx, lb_cand.path_x.size() - 1);
        int ref_idx = idxs[sample_idx];
        double dx = lb_cand.path_x[sample_idx] - ref_->x()[ref_idx];
        double dy = lb_cand.path_y[sample_idx] - ref_->y()[ref_idx];
        avg_lat_offset = std::sqrt(dx * dx + dy * dy);
        if (lb_cand.corner_sign > 0) avg_lat_offset = -avg_lat_offset;  // 우회전 시 바깥 = 음수
      }

      // Compute reward
      double R = computeReward(traj, lb_kappa, avg_lat_offset,
                               opp_future_x, opp_future_y, opp_future_yaw);

      // Late braking 보너스: 추월 시도에 보상
      R += config_.weights.w_overtake * 2.0;

      // 브레이킹 포인트가 늦을수록 (delay가 높을수록) 보너스
      double delay_bonus = static_cast<double>(lb_cand.brake_point_idx) /
                           static_cast<double>(lb_cand.apex_idx + 1);
      R += config_.weights.w_overtake * delay_bonus * 1.5;

      // Hysteresis 적용
      if (has_prev_offset_) {
        double offset_change = std::abs(avg_lat_offset - prev_lat_offset_);
        R -= config_.lat_change_penalty * offset_change * 0.5;  // late braking은 페널티 감소
      }

      // Side commitment (late braking은 인사이드로 가므로)
      if (committed_side_ != 0 && committed_side_ != corner_sign) {
        R -= config_.overtake_side_commit_penalty * 0.3;  // late braking은 페널티 감소
      }

      // Phase 보너스
      if (phase_execute) {
        R += config_.strategy.overtake.gap_reward_scale * 2.5;  // 실행 단계에서 late braking 적극 권장
      }

      debug_results.push_back({R, avg_lat_offset, 1.0});

      if (R > best.reward) {
        best.reward = R;
        best.lat_offset = avg_lat_offset;
        best.speed_scale = 1.0;
        best.trajectory = std::move(traj);
        best.candidate_path.clear();
        for (size_t i = 0; i < lb_cand.path_x.size(); ++i) {
          best.candidate_path.emplace_back(lb_cand.path_x[i], lb_cand.path_y[i]);
        }
        best.v_desired = lb_cand.v_desired;
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

  // Update side commitment state
  if (overtake_active) {
    if (!was_overtaking_) {
      // Overtake just started: commit to the chosen side
      if (best.lat_offset > 0.1) {
        committed_side_ = 1;
      } else if (best.lat_offset < -0.1) {
        committed_side_ = -1;
      }
    }
    was_overtaking_ = true;
  } else {
    // Overtake ended: reset commitment
    if (was_overtaking_) {
      committed_side_ = 0;
    }
    was_overtaking_ = false;
  }

  return best;
}

}  // namespace scenario_director
