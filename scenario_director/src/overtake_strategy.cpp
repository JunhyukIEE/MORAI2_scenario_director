#include "scenario_director/overtake_strategy.hpp"

#include <algorithm>
#include <cmath>

namespace scenario_director {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-6;

double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

OvertakeStrategyPlanner::OvertakeStrategyPlanner(OvertakeStrategyConfig config)
    : config_(config) {}

void OvertakeStrategyPlanner::setConfig(const OvertakeStrategyConfig &config) {
  config_ = config;
}

const OvertakeStrategyConfig &OvertakeStrategyPlanner::config() const {
  return config_;
}

CornerInfo OvertakeStrategyPlanner::analyzeCorners(const RacingLine &line,
                                                    const VehicleState &ego) const {
  CornerInfo info;
  const auto &points = line.points();
  if (points.size() < 3) {
    return info;
  }

  const int n = static_cast<int>(points.size());
  const int nearest_idx = line.getNearestIndex(ego.x, ego.y);

  // 첫 번째 코너 (apex) 찾기
  double max_curvature = 0.0;
  double traveled = 0.0;

  for (int step = 0; step < n && traveled < config_.apex_search_distance; ++step) {
    const int idx = (nearest_idx + step) % n;
    const double curv = computeCurvature(points, idx);

    if (curv > max_curvature && curv > config_.min_corner_curvature) {
      max_curvature = curv;
      info.apex_idx = idx;
      info.curvature = curv;
      info.distance_to_apex = traveled;

      // 코너 방향 판단 (yaw 변화로)
      const int prev_idx = (idx - 1 + n) % n;
      const int next_idx = (idx + 1) % n;
      const double dyaw = normalizeAngle(points[next_idx].yaw - points[prev_idx].yaw);
      info.is_left_turn = dyaw > 0;
    }

    if (step > 0) {
      const int prev = (idx - 1 + n) % n;
      const double dx = points[idx].x - points[prev].x;
      const double dy = points[idx].y - points[prev].y;
      traveled += std::sqrt(dx * dx + dy * dy);
    }
  }

  if (info.apex_idx < 0) {
    return info;
  }

  // 두 번째 코너 찾기 (연속 코너 감지용)
  double second_max_curv = 0.0;
  double traveled_from_first = 0.0;
  int search_start = (info.apex_idx + 5) % n;  // 첫 apex 이후부터 검색

  for (int step = 0; step < n && traveled_from_first < config_.chicane_detection_dist; ++step) {
    const int idx = (search_start + step) % n;
    const double curv = computeCurvature(points, idx);

    if (curv > second_max_curv && curv > config_.min_corner_curvature) {
      second_max_curv = curv;
      info.next_apex_idx = idx;
      info.distance_to_next_apex = info.distance_to_apex + traveled_from_first;

      const int prev_idx = (idx - 1 + n) % n;
      const int next_idx = (idx + 1) % n;
      const double dyaw = normalizeAngle(points[next_idx].yaw - points[prev_idx].yaw);
      info.next_is_left_turn = dyaw > 0;
    }

    if (step > 0) {
      const int prev = (search_start + step - 1 + n) % n;
      const double dx = points[idx].x - points[prev].x;
      const double dy = points[idx].y - points[prev].y;
      traveled_from_first += std::sqrt(dx * dx + dy * dy);
    }
  }

  // S자 코너 (chicane) 판단: 두 코너 방향이 반대
  if (info.next_apex_idx >= 0) {
    info.is_chicane = (info.is_left_turn != info.next_is_left_turn);
  }

  return info;
}

double OvertakeStrategyPlanner::computeCurvature(const std::vector<Waypoint> &points,
                                                  int idx) const {
  const int n = static_cast<int>(points.size());
  if (n < 3) {
    return 0.0;
  }

  const int prev_idx = (idx - 1 + n) % n;
  const int next_idx = (idx + 1) % n;

  const double dx1 = points[idx].x - points[prev_idx].x;
  const double dy1 = points[idx].y - points[prev_idx].y;
  const double dx2 = points[next_idx].x - points[idx].x;
  const double dy2 = points[next_idx].y - points[idx].y;

  const double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
  const double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

  if (len1 < kEpsilon || len2 < kEpsilon) {
    return 0.0;
  }

  const double dyaw = normalizeAngle(points[next_idx].yaw - points[prev_idx].yaw);
  return std::abs(dyaw) / (len1 + len2);
}

StrategyResult OvertakeStrategyPlanner::planStrategy(
    const RacingLine &line,
    const VehicleState &ego,
    const VehicleState &opponent,
    DrivingState driving_state,
    const std::string &current_overtake_side) {

  StrategyResult result;
  result.recommended_line = current_overtake_side.empty() ? "optimal" : current_overtake_side;

  // NORMAL 상태에서는 전략 없음
  if (driving_state == DrivingState::NORMAL) {
    active_strategy_ = OvertakeStrategyType::NONE;
    dummy_active_ = false;
    dummy_frame_count_ = 0;
    return result;
  }

  // 코너 분석
  const CornerInfo corner = analyzeCorners(line, ego);

  // 상대와의 거리/위치 계산
  (void)opponent;  // 향후 확장을 위해 예약

  // 각 전략의 적합성 평가
  const double late_brake_score = evaluateLateBraking(corner, ego, opponent);
  const double dummy_score = evaluateDummy(corner, ego, opponent);
  const double outside_score = evaluateAroundOutside(corner, ego, opponent);

  // 가장 적합한 전략 선택
  double max_score = 0.0;
  OvertakeStrategyType best_strategy = OvertakeStrategyType::SLOW_IN_FAST_OUT;

  if (late_brake_score > max_score) {
    max_score = late_brake_score;
    best_strategy = OvertakeStrategyType::LATE_BRAKING;
  }
  if (dummy_score > max_score) {
    max_score = dummy_score;
    best_strategy = OvertakeStrategyType::DUMMY;
  }
  if (outside_score > max_score) {
    max_score = outside_score;
    best_strategy = OvertakeStrategyType::AROUND_OUTSIDE;
  }

  // 이미 활성화된 전략이 있으면 계속 유지 (갑작스러운 전략 변경 방지)
  if (active_strategy_ != OvertakeStrategyType::NONE &&
      active_strategy_ != OvertakeStrategyType::SLOW_IN_FAST_OUT) {
    best_strategy = active_strategy_;
  }

  // 선택된 전략 실행
  switch (best_strategy) {
    case OvertakeStrategyType::LATE_BRAKING:
      result = executeLateBraking(corner, ego, opponent);
      break;
    case OvertakeStrategyType::DUMMY:
      result = executeDummy(corner, ego, opponent);
      break;
    case OvertakeStrategyType::AROUND_OUTSIDE:
      result = executeAroundOutside(corner, ego, opponent);
      break;
    default:
      // SLOW_IN_FAST_OUT은 SlowInOutPlanner에서 처리
      result.active_strategy = OvertakeStrategyType::SLOW_IN_FAST_OUT;
      break;
  }

  active_strategy_ = result.active_strategy;
  return result;
}

double OvertakeStrategyPlanner::evaluateLateBraking(const CornerInfo &corner,
                                                     const VehicleState &ego,
                                                     const VehicleState &opponent) const {
  // Late Braking 적합성 평가
  // - 코너가 가까울수록 높은 점수
  // - 상대보다 빠를 때 높은 점수
  // - 인사이드로 진입 가능할 때

  if (corner.apex_idx < 0) {
    return 0.0;
  }

  double score = 0.0;

  // 코너까지 거리가 적절한 범위 (late_brake_distance ~ 3*late_brake_distance)
  if (corner.distance_to_apex > config_.late_brake_distance &&
      corner.distance_to_apex < config_.late_brake_distance * 3.0) {
    score += 0.4;
  }

  // 속도 차이
  const double speed_diff = ego.speed - opponent.speed;
  if (speed_diff > 2.0) {
    score += 0.3 * std::min(speed_diff / 5.0, 1.0);
  }

  // 코너 곡률이 클수록 late braking이 효과적
  if (corner.curvature > 0.1) {
    score += 0.3;
  }

  return score;
}

double OvertakeStrategyPlanner::evaluateDummy(const CornerInfo &corner,
                                               const VehicleState &ego,
                                               const VehicleState &opponent) const {
  // Dummy(페인트) 적합성 평가
  // - 직선 구간에서 효과적
  // - 상대와의 거리가 적절할 때

  const double dx = opponent.x - ego.x;
  const double dy = opponent.y - ego.y;
  const double rel_dist = std::sqrt(dx * dx + dy * dy);

  double score = 0.0;

  // 코너가 멀거나 없을 때 (직선 구간)
  if (corner.apex_idx < 0 || corner.distance_to_apex > config_.dummy_trigger_distance * 1.5) {
    score += 0.4;
  }

  // 적절한 거리
  if (rel_dist > 10.0 && rel_dist < config_.dummy_trigger_distance) {
    score += 0.3;
  }

  // 속도가 비슷할 때 페인트가 효과적
  const double speed_diff = std::abs(ego.speed - opponent.speed);
  if (speed_diff < 3.0) {
    score += 0.3;
  }

  return score;
}

double OvertakeStrategyPlanner::evaluateAroundOutside(const CornerInfo &corner,
                                                       const VehicleState &ego,
                                                       const VehicleState &opponent) const {
  // Around the Outside 적합성 평가
  // - S자 코너(chicane)에서 매우 효과적
  // - 연속 코너에서 효과적

  if (corner.apex_idx < 0) {
    return 0.0;
  }

  double score = 0.0;

  // S자 코너 (가장 적합)
  if (corner.is_chicane) {
    score += 0.6;
  }

  // 연속 코너가 있을 때
  if (corner.next_apex_idx >= 0 && !corner.is_chicane) {
    score += 0.3;
  }

  // 적절한 거리
  if (corner.distance_to_apex > 15.0 && corner.distance_to_apex < 40.0) {
    score += 0.2;
  }

  // 속도 우위가 있을 때
  if (ego.speed > opponent.speed) {
    score += 0.2;
  }

  return score;
}

StrategyResult OvertakeStrategyPlanner::executeLateBraking(const CornerInfo &corner,
                                                            const VehicleState & /*ego*/,
                                                            const VehicleState & /*opponent*/) {
  StrategyResult result;
  result.active_strategy = OvertakeStrategyType::LATE_BRAKING;

  // 코너 방향에 따라 인사이드 결정
  // 왼쪽 코너면 인사이드는 왼쪽 (inside), 오른쪽 코너면 인사이드는 오른쪽 (outside)
  if (corner.is_left_turn) {
    result.recommended_line = "inside";
  } else {
    result.recommended_line = "outside";
  }

  // 브레이킹 포인트에 따른 속도 조절
  if (corner.distance_to_apex < config_.late_brake_distance) {
    // 브레이킹 구간 - 늦게 브레이킹하되 확실하게
    result.speed_modifier = config_.late_brake_speed_scale;
  } else if (corner.distance_to_apex < config_.late_brake_distance * 2.0) {
    // 브레이킹 직전 - 최대 속도 유지 (상대보다 늦게 브레이킹)
    result.speed_modifier = 1.05;
  } else {
    // 아직 멀리 있음 - 정상 속도
    result.speed_modifier = 1.0;
  }

  return result;
}

StrategyResult OvertakeStrategyPlanner::executeDummy(const CornerInfo & /*corner*/,
                                                      const VehicleState &ego,
                                                      const VehicleState &opponent) {
  StrategyResult result;
  result.active_strategy = OvertakeStrategyType::DUMMY;

  const double dx = opponent.x - ego.x;
  const double dy = opponent.y - ego.y;

  // 상대 위치 (ego 기준 왼쪽/오른쪽)
  const double left_x = -std::sin(ego.yaw);
  const double left_y = std::cos(ego.yaw);
  const double cross = dx * left_x + dy * left_y;
  const bool opponent_on_left = cross > 0;

  if (!dummy_active_) {
    // 페인트 시작: 상대 반대편으로 먼저 움직이는 척
    dummy_active_ = true;
    dummy_direction_left_ = opponent_on_left;  // 상대 있는 쪽으로 페인트
    dummy_frame_count_ = 0;
    last_opponent_lateral_ = cross;
  }

  dummy_frame_count_++;

  // 페인트 동작 시간 (약 0.5초, 20Hz 기준 10프레임)
  const int dummy_frames = static_cast<int>(config_.dummy_duration * 20.0);

  if (dummy_frame_count_ < dummy_frames) {
    // 페인트 동작 중 - 상대 방향으로 움직이는 척
    result.execute_dummy = true;
    result.dummy_direction_left = dummy_direction_left_;
    result.lateral_offset = dummy_direction_left_ ? config_.dummy_offset : -config_.dummy_offset;

    // 라인은 페인트 방향
    result.recommended_line = dummy_direction_left_ ? "inside" : "outside";
  } else {
    // 페인트 완료 - 반대편으로 실제 추월
    result.execute_dummy = false;

    // 상대가 반응했는지 확인
    const bool opponent_reacted = isOpponentReactingToDummy(ego, opponent);

    if (opponent_reacted || dummy_frame_count_ > dummy_frames * 2) {
      // 상대가 반응했거나 충분히 기다렸으면 반대편으로
      result.recommended_line = dummy_direction_left_ ? "outside" : "inside";
      result.speed_modifier = 1.1;  // 가속

      // 페인트 완료 후 리셋
      if (dummy_frame_count_ > dummy_frames * 3) {
        dummy_active_ = false;
      }
    } else {
      // 상대가 반응 안 함 - 계속 페인트 방향으로 추월
      result.recommended_line = dummy_direction_left_ ? "inside" : "outside";
    }
  }

  return result;
}

StrategyResult OvertakeStrategyPlanner::executeAroundOutside(const CornerInfo &corner,
                                                              const VehicleState & /*ego*/,
                                                              const VehicleState & /*opponent*/) {
  StrategyResult result;
  result.active_strategy = OvertakeStrategyType::AROUND_OUTSIDE;

  // 첫 코너 기준 바깥쪽 선택
  // 왼쪽 코너면 바깥쪽은 오른쪽 (outside), 오른쪽 코너면 바깥쪽은 왼쪽 (inside)
  if (corner.is_left_turn) {
    result.recommended_line = "outside";
  } else {
    result.recommended_line = "inside";
  }

  // 진입 시 속도 조절
  if (corner.distance_to_apex > 15.0) {
    // 진입 전 - 상대와 나란히 유지
    result.speed_modifier = config_.outside_entry_speed_scale;
  } else if (corner.distance_to_apex > 5.0) {
    // apex 접근 - 상대보다 늦게 브레이킹
    result.speed_modifier = 0.95;
  } else {
    // apex 통과 후 - 가속하여 다음 코너에서 유리한 위치
    result.speed_modifier = config_.outside_exit_boost;
  }

  // S자 코너에서는 다음 코너를 위해 추가 가속
  if (corner.is_chicane && corner.distance_to_apex < 10.0) {
    // 첫 코너 apex 근처에서 가속 시작
    result.speed_modifier = config_.outside_exit_boost;
  }

  return result;
}

bool OvertakeStrategyPlanner::isOpponentReactingToDummy(const VehicleState &ego,
                                                         const VehicleState &opponent) const {
  // 상대가 페인트에 반응했는지 판단
  // 상대의 횡방향 움직임 확인
  const double dx = opponent.x - ego.x;
  const double dy = opponent.y - ego.y;
  const double left_x = -std::sin(ego.yaw);
  const double left_y = std::cos(ego.yaw);
  const double current_lateral = dx * left_x + dy * left_y;

  const double lateral_change = std::abs(current_lateral - last_opponent_lateral_);

  // 상대가 페인트 방향으로 움직였으면 반응한 것
  return lateral_change > config_.dummy_switch_threshold;
}

OvertakeStrategyType OvertakeStrategyPlanner::getActiveStrategy() const {
  return active_strategy_;
}

std::string OvertakeStrategyPlanner::strategyToString(OvertakeStrategyType type) {
  switch (type) {
    case OvertakeStrategyType::NONE:
      return "NONE";
    case OvertakeStrategyType::SLOW_IN_FAST_OUT:
      return "SLOW_IN_FAST_OUT";
    case OvertakeStrategyType::LATE_BRAKING:
      return "LATE_BRAKING";
    case OvertakeStrategyType::DUMMY:
      return "DUMMY";
    case OvertakeStrategyType::AROUND_OUTSIDE:
      return "AROUND_OUTSIDE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace scenario_director
