#include "scenario_director/slow_in_out.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace scenario_director {

namespace {

constexpr double kEpsilon = 1e-6;

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double forwardDistance(const std::vector<Waypoint> &points, int start_idx, int end_idx) {
  if (points.empty() || start_idx < 0 || end_idx < 0) {
    return 0.0;
  }
  const int n = static_cast<int>(points.size());
  if (n == 0 || start_idx == end_idx) {
    return 0.0;
  }

  double dist = 0.0;
  int idx = start_idx;
  for (int i = 0; i < n && idx != end_idx; ++i) {
    const int next = (idx + 1) % n;
    const double dx = points[next].x - points[idx].x;
    const double dy = points[next].y - points[idx].y;
    dist += std::sqrt(dx * dx + dy * dy);
    idx = next;
  }
  return dist;
}

}  // namespace

SlowInOutPlanner::SlowInOutPlanner(SlowInOutConfig config)
: config_(config) {}

void SlowInOutPlanner::setConfig(const SlowInOutConfig &config) {
  config_ = config;
}

const SlowInOutConfig &SlowInOutPlanner::config() const {
  return config_;
}

CornerAnalysis SlowInOutPlanner::analyzeCorner(const RacingLine &line,
                                                const VehicleState &ego) const {
  CornerAnalysis result;
  const auto &points = line.points();
  if (points.size() < 3) {
    return result;
  }

  const int n = static_cast<int>(points.size());
  const int nearest_idx = line.getNearestIndex(ego.x, ego.y);

  double max_curvature = 0.0;
  double traveled = 0.0;

  for (int step = 0; step < n && traveled < config_.apex_search_distance; ++step) {
    const int idx1 = (nearest_idx + step) % n;
    const int idx2 = (idx1 + 1) % n;

    const double dx = points[idx2].x - points[idx1].x;
    const double dy = points[idx2].y - points[idx1].y;
    const double seg_len = std::sqrt(dx * dx + dy * dy);

    if (seg_len > kEpsilon) {
      const double dyaw = normalizeAngle(points[idx2].yaw - points[idx1].yaw);
      const double curvature = std::abs(dyaw) / seg_len;

      if (curvature > max_curvature) {
        max_curvature = curvature;
        result.apex_idx = idx2;
        result.max_curvature = curvature;
        result.distance_to_apex = traveled;
        result.is_left_turn = dyaw > 0;
      }
      traveled += seg_len;
    } else {
      traveled += seg_len;
    }
  }

  if (result.apex_idx >= 0 && max_curvature >= config_.curvature_threshold) {
    result.in_entry_zone = result.distance_to_apex <= config_.entry_distance;
    result.in_exit_zone = false;

    if (!result.in_entry_zone) {
      const double dist_from_apex = forwardDistance(points, result.apex_idx, nearest_idx);
      result.in_exit_zone = dist_from_apex <= config_.exit_distance;
    }
  }

  return result;
}

double SlowInOutPlanner::getOptimalBrakingDistance(const RacingLine &line,
                                                    const VehicleState &ego,
                                                    double current_speed) const {
  const CornerAnalysis corner = analyzeCorner(line, ego);

  if (corner.apex_idx < 0) {
    return 0.0;  // 코너 없음
  }

  // 곡률에 따른 목표 코너 속도 계산
  // 높은 곡률 = 낮은 목표 속도
  const double max_lateral_accel = 8.0;  // m/s^2
  double target_corner_speed = std::sqrt(max_lateral_accel / std::max(corner.max_curvature, 0.01));
  target_corner_speed = std::min(target_corner_speed, current_speed);

  // 필요한 감속량
  const double speed_reduction = current_speed - target_corner_speed;
  if (speed_reduction <= 0) {
    return 0.0;
  }

  // 브레이킹 거리 계산 (운동역학)
  // v^2 = u^2 + 2as => s = (u^2 - v^2) / (2a)
  const double decel = 6.0;  // m/s^2 감속도
  const double brake_distance = (current_speed * current_speed -
                                  target_corner_speed * target_corner_speed) / (2.0 * decel);

  return brake_distance;
}

double SlowInOutPlanner::adjustTargetSpeed(const RacingLine &line,
                                           const VehicleState &ego,
                                           double base_speed,
                                           bool is_overtaking) const {
  if (!config_.enabled || base_speed <= 0.0) {
    return base_speed;
  }

  const auto &points = line.points();
  if (points.size() < 3) {
    return base_speed;
  }

  const int n = static_cast<int>(points.size());
  const int nearest_idx = line.getNearestIndex(ego.x, ego.y);

  double max_curvature = 0.0;
  int apex_idx = -1;
  double traveled = 0.0;

  for (int step = 0; step < n && traveled < config_.apex_search_distance; ++step) {
    const int idx1 = (nearest_idx + step) % n;
    const int idx2 = (idx1 + 1) % n;

    const double dx = points[idx2].x - points[idx1].x;
    const double dy = points[idx2].y - points[idx1].y;
    const double seg_len = std::sqrt(dx * dx + dy * dy);
    if (seg_len > kEpsilon) {
      const double dyaw = normalizeAngle(points[idx2].yaw - points[idx1].yaw);
      const double curvature = std::abs(dyaw) / seg_len;
      if (curvature > max_curvature) {
        max_curvature = curvature;
        apex_idx = idx2;
      }
      traveled += seg_len;
    } else {
      traveled += seg_len;
    }
  }

  if (apex_idx < 0 || max_curvature < config_.curvature_threshold) {
    return base_speed;
  }

  const double dist_to_apex = forwardDistance(points, nearest_idx, apex_idx);

  // 추월 상황에서의 속도 조절
  double entry_scale = config_.entry_speed_scale;
  double exit_scale = config_.exit_speed_scale;

  if (is_overtaking && config_.aggressive_exit) {
    // 추월 시: 더 늦게 브레이킹, 더 빠르게 가속
    entry_scale = config_.overtake_entry_speed_scale;
    exit_scale = config_.overtake_exit_speed_scale;
  }

  // 진입 구간 (Entry Zone)
  if (dist_to_apex <= config_.entry_distance) {
    // 거리에 따른 점진적 감속
    // dist_to_apex가 0에 가까울수록 entry_scale, 멀수록 1.0
    const double t = config_.entry_distance > kEpsilon
                       ? (dist_to_apex / config_.entry_distance)
                       : 0.0;
    const double factor = entry_scale + (1.0 - entry_scale) * std::clamp(t, 0.0, 1.0);
    return base_speed * factor;
  }

  // 탈출 구간 (Exit Zone) - apex 이후
  const double dist_from_apex = forwardDistance(points, apex_idx, nearest_idx);
  if (dist_from_apex <= config_.exit_distance && dist_from_apex > 0) {
    // apex에서 멀어질수록 가속
    // dist_from_apex가 0에 가까울수록 1.0, 멀수록 exit_scale
    const double t = config_.exit_distance > kEpsilon
                       ? (dist_from_apex / config_.exit_distance)
                       : 1.0;
    const double factor = 1.0 + (exit_scale - 1.0) * std::clamp(t, 0.0, 1.0);
    return base_speed * factor;
  }

  return base_speed;
}

}  // namespace scenario_director
