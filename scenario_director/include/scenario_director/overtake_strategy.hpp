#pragma once

#include "scenario_director/line_manager.hpp"
#include "scenario_director/waypoint_selector.hpp"

#include <memory>
#include <string>
#include <vector>

namespace scenario_director {

// 추월 전략 유형
enum class OvertakeStrategyType {
  NONE,
  SLOW_IN_FAST_OUT,  // 코너에서 늦게 들어가고 빠르게 나가기
  LATE_BRAKING,      // 늦은 브레이킹으로 인사이드 블록 패스
  DUMMY,             // 페인트 동작으로 상대 속이기
  AROUND_OUTSIDE     // 바깥쪽으로 추월 (연속 코너에서 유리)
};

// 코너 정보
struct CornerInfo {
  int apex_idx = -1;
  double distance_to_apex = 0.0;
  double curvature = 0.0;
  bool is_left_turn = true;  // 왼쪽 코너인지

  // 연속 코너 정보
  int next_apex_idx = -1;
  double distance_to_next_apex = 0.0;
  bool next_is_left_turn = true;
  bool is_chicane = false;  // S자 코너인지 (방향이 반대인 연속 코너)
};

// 전략 결과
struct StrategyResult {
  OvertakeStrategyType active_strategy = OvertakeStrategyType::NONE;
  std::string recommended_line = "optimal";
  double speed_modifier = 1.0;      // 속도 배율
  double lateral_offset = 0.0;      // 횡방향 오프셋 (페인트용)
  bool execute_dummy = false;       // 페인트 동작 실행
  bool dummy_direction_left = true; // 페인트 방향
};

// 전략 설정
struct OvertakeStrategyConfig {
  // Late Braking 설정
  double late_brake_distance = 8.0;      // 브레이킹 시작 거리
  double late_brake_speed_scale = 0.75;  // 늦은 브레이킹 시 속도 감소율
  double block_pass_offset = 2.5;        // 인사이드 진입 오프셋

  // Dummy 설정
  double dummy_trigger_distance = 20.0;  // 페인트 시작 거리
  double dummy_offset = 1.5;             // 페인트 횡방향 이동량
  double dummy_duration = 0.5;           // 페인트 유지 시간 (초)
  double dummy_switch_threshold = 0.8;   // 상대가 반응했다고 판단하는 임계값

  // Around Outside 설정
  double outside_entry_speed_scale = 0.9;  // 바깥쪽 진입 속도
  double outside_exit_boost = 1.15;        // 바깥쪽 탈출 가속
  double chicane_detection_dist = 40.0;    // 연속 코너 감지 거리

  // 공통 설정
  double min_corner_curvature = 0.05;      // 코너로 인식하는 최소 곡률
  double apex_search_distance = 50.0;      // apex 검색 거리
};

class OvertakeStrategyPlanner {
public:
  explicit OvertakeStrategyPlanner(OvertakeStrategyConfig config = {});

  void setConfig(const OvertakeStrategyConfig &config);
  const OvertakeStrategyConfig &config() const;

  // 현재 상황 분석 및 전략 결정
  StrategyResult planStrategy(
      const RacingLine &line,
      const VehicleState &ego,
      const VehicleState &opponent,
      DrivingState driving_state,
      const std::string &current_overtake_side);

  // 코너 정보 분석
  CornerInfo analyzeCorners(const RacingLine &line, const VehicleState &ego) const;

  // 현재 활성화된 전략
  OvertakeStrategyType getActiveStrategy() const;

  // 디버그용: 전략 이름 반환
  static std::string strategyToString(OvertakeStrategyType type);

private:
  // 각 전략 평가
  double evaluateLateBraking(const CornerInfo &corner, const VehicleState &ego,
                              const VehicleState &opponent) const;
  double evaluateDummy(const CornerInfo &corner, const VehicleState &ego,
                       const VehicleState &opponent) const;
  double evaluateAroundOutside(const CornerInfo &corner, const VehicleState &ego,
                                const VehicleState &opponent) const;

  // 각 전략 실행
  StrategyResult executeLateBraking(const CornerInfo &corner, const VehicleState &ego,
                                     const VehicleState &opponent);
  StrategyResult executeDummy(const CornerInfo &corner, const VehicleState &ego,
                               const VehicleState &opponent);
  StrategyResult executeAroundOutside(const CornerInfo &corner, const VehicleState &ego,
                                       const VehicleState &opponent);

  // 유틸리티
  double computeCurvature(const std::vector<Waypoint> &points, int idx) const;
  bool isOpponentReactingToDummy(const VehicleState &ego, const VehicleState &opponent) const;

  OvertakeStrategyConfig config_;
  OvertakeStrategyType active_strategy_ = OvertakeStrategyType::NONE;

  // Dummy 상태
  bool dummy_active_ = false;
  bool dummy_direction_left_ = true;
  int dummy_frame_count_ = 0;
  double last_opponent_lateral_ = 0.0;
};

}  // namespace scenario_director
