#pragma once

#include "scenario_director/line_manager.hpp"
#include "scenario_director/waypoint_selector.hpp"

namespace scenario_director {

struct SlowInOutConfig {
  bool enabled = true;
  double apex_search_distance = 25.0;
  double entry_distance = 12.0;
  double exit_distance = 12.0;
  double entry_speed_scale = 0.85;
  double exit_speed_scale = 1.05;
  double curvature_threshold = 0.08;

  // 추월 상황에서의 추가 설정
  bool aggressive_exit = true;          // 추월 시 더 공격적인 탈출 가속
  double overtake_exit_speed_scale = 1.15;  // 추월 시 탈출 속도 배율
  double overtake_entry_speed_scale = 0.80; // 추월 시 진입 속도 배율 (더 늦게 브레이킹)
};

// 코너 분석 결과
struct CornerAnalysis {
  int apex_idx = -1;
  double distance_to_apex = 0.0;
  double max_curvature = 0.0;
  bool in_entry_zone = false;
  bool in_exit_zone = false;
  bool is_left_turn = true;
};

class SlowInOutPlanner {
public:
  explicit SlowInOutPlanner(SlowInOutConfig config = {});

  void setConfig(const SlowInOutConfig &config);
  const SlowInOutConfig &config() const;

  // 기본 속도 조절 (추월 상태 고려)
  double adjustTargetSpeed(const RacingLine &line,
                           const VehicleState &ego,
                           double base_speed,
                           bool is_overtaking = false) const;

  // 코너 분석
  CornerAnalysis analyzeCorner(const RacingLine &line,
                                const VehicleState &ego) const;

  // 추월 시 최적 브레이킹 포인트 계산
  double getOptimalBrakingDistance(const RacingLine &line,
                                    const VehicleState &ego,
                                    double current_speed) const;

private:
  SlowInOutConfig config_;
};

}  // namespace scenario_director
