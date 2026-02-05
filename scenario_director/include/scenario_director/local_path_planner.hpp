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
  double wheelbase = 2.35;
  double max_steer = 0.489;
  double max_accel = 3.5;
  double max_decel = 7.0;
  double max_lat_accel = 8.5;
};

struct PlannerWeights {
  double w_progress = 8.0;
  double w_speed = 1.5;
  double w_overtake = 60.0;
  double w_collision = 20000.0;
  double w_smooth = 8.0;
  double w_curv = 2.0;
  double w_far_from_ref = 2.0;
};

struct LateBrakingConfig {
  bool enabled = true;
  double delay_factor = 0.3;              // 브레이킹 포인트를 얼마나 늦출지 (0.0~1.0, 높을수록 늦게)
  double outside_offset = 2.5;            // 브레이킹 전 바깥쪽 오프셋 (m)
  double inside_offset = -1.5;            // apex에서 안쪽 오프셋 (m), 코너 방향에 따라 부호 반전됨
  double transition_sharpness = 2.0;      // 전환 곡선의 날카로움 (높을수록 급격히 꺾음)
  double min_corner_curvature = 0.03;     // late braking 적용할 최소 곡률
  double safety_margin = 1.2;             // 브레이킹 거리 안전 마진 (1.0 = 마진 없음)
  double max_decel_override = 0.0;        // 0이면 vehicle.max_decel 사용, 아니면 이 값 사용
  double exit_recovery_distance = 15.0;   // apex 후 레이싱라인 복귀 거리 (m)
};

struct OvertakeStrategyConfig {
  // Late braking 설정 (브레이킹 포인트 기반)
  LateBrakingConfig late_braking;

  double min_corner_curvature = 0.05;
  double apex_search_distance = 50.0;

  // Phase-based overtake parameters
  double approach_speed_boost = 1.05;       // 접근 단계 속도 부스트
  double position_lateral_offset = 2.0;     // 위치 선점 시 목표 lateral offset
  double execute_speed_boost = 1.10;        // 실행 단계 속도 부스트
  double slipstream_distance = 8.0;         // 슬립스트림 효과 거리
  double slipstream_speed_boost = 1.08;     // 슬립스트림 속도 부스트
  double gap_reward_scale = 200.0;          // 갭 찾기 보상 스케일

  // 동적 회피 경로 설정
  double avoidance_path_smoothness = 2.0;   // 경로 부드러움 (1.0~3.0)
  std::vector<double> avoidance_offsets = {2.0, 2.5, 3.0, 3.5};  // 회피 offset 후보
};

struct StrategyConfig {
  OvertakeStrategyConfig overtake;
};

struct LocalPlannerConfig {
  VehicleParams vehicle;
  PlannerWeights weights;

  double ego_length = 4.0;
  double ego_width = 1.7;
  double opp_length = 4.0;
  double opp_width = 1.7;

  // 갭 통과 안전 마진
  double pass_gap_margin = 0.8;           // 통과 시 필요한 추가 마진 (m)
  double gap_check_distance = 30.0;       // 갭 체크할 전방 거리 (m)
  double impassable_penalty = 50000.0;    // 통과 불가능 방향 페널티

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

  // Side commitment during overtake: extra penalty for switching sides
  double overtake_side_commit_penalty = 3000.0;  // Penalty for switching sides during overtake

  StrategyConfig strategy;
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
  std::vector<std::pair<double, double>> candidate_path;  // (x, y) 선택된 경로
  std::vector<double> v_desired;

  struct DebugCandidate {
    double reward;
    double lat_offset;
    double speed_scale;
    std::vector<std::pair<double, double>> path;  // 후보 경로 점들
    bool is_late_braking = false;  // Late Braking 경로 여부
  };
  std::vector<DebugCandidate> debug_top;

  // 모든 후보 경로 (시각화용)
  std::vector<DebugCandidate> all_candidates;
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

  // Late braking 경로 생성: 브레이킹 포인트 기반 가변 오프셋
  // brake_point_idx: 브레이킹 시작 인덱스 (indices 내 상대 인덱스)
  // apex_idx: 코너 apex 인덱스 (indices 내 상대 인덱스)
  // corner_sign: 코너 방향 (+1: 우회전, -1: 좌회전)
  // outside_offset: 브레이킹 전 바깥쪽 오프셋
  // inside_offset: apex에서 안쪽 오프셋 (절대값, 코너 방향에 따라 부호 결정)
  // sharpness: 전환 곡선 날카로움 (1.0=선형, 높을수록 급격)
  // exit_distance: apex 후 복귀 거리
  void getLateBrakingLine(const std::vector<int>& indices,
                          size_t brake_point_idx, size_t apex_idx,
                          int corner_sign,
                          double outside_offset, double inside_offset,
                          double sharpness, double exit_distance,
                          std::vector<double>& out_x, std::vector<double>& out_y) const;

  // 상대 차량 회피 경로 생성
  // opp_idx: 상대 차량 위치의 인덱스 (indices 내 상대 인덱스)
  // pass_side: 통과 방향 (+1: 오른쪽, -1: 왼쪽)
  // pass_offset: 통과 시 lateral offset (m)
  // entry_distance: 회피 시작 거리 (상대 앞 몇 m에서 시작)
  // exit_distance: 회피 종료 거리 (상대 뒤 몇 m에서 복귀)
  // smoothness: 전환 부드러움 (높을수록 부드러움, 1.0~3.0)
  void getAvoidancePath(const std::vector<int>& indices,
                        size_t opp_idx, int pass_side, double pass_offset,
                        double entry_distance, double exit_distance,
                        double smoothness,
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

  // 기존 방식 (등속 가정) - deprecated
  void predictStates(double opp_x, double opp_y, double opp_v,
                     double horizon_s, double dt,
                     std::vector<double>& out_x,
                     std::vector<double>& out_y,
                     std::vector<double>& out_yaw) const;

  // 개선된 방식: v_ref × speed_ratio 기반 예측
  // speed_ratio: NPC의 속도 배율 (0이면 현재 속도에서 자동 추정)
  void predictStatesWithVRef(double opp_x, double opp_y, double opp_v,
                              double speed_ratio,
                              double horizon_s, double dt,
                              std::vector<double>& out_x,
                              std::vector<double>& out_y,
                              std::vector<double>& out_yaw,
                              std::vector<double>& out_v) const;

  // 현재 속도로부터 speed_ratio 추정
  double estimateSpeedRatio(double opp_x, double opp_y, double opp_v) const;

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

  // overtake_phase: 0=NONE, 1=APPROACH, 2=POSITION, 3=EXECUTE, 4=COMPLETE
  PlanResult plan(double ego_x, double ego_y, double ego_yaw, double ego_v,
                  double opp_x, double opp_y, double opp_v,
                  bool overtake_flag, int overtake_phase = 0);

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

  // Late braking: 브레이킹 포인트 계산
  // current_speed: 현재 속도 (m/s)
  // apex_curvature: apex 곡률 (1/m)
  // delay_factor: 브레이킹 지연 계수 (0.0~1.0)
  // 반환: apex까지의 필요 브레이킹 거리 (m)
  double computeBrakingDistance(double current_speed, double apex_curvature,
                                double delay_factor) const;

  // 상대 차량 옆 통과 가능 여부 체크
  // 상대 차량과 트랙 경계 사이의 갭이 ego 차량이 지나갈 수 있는지 확인
  // 반환: {left_passable, right_passable, left_gap, right_gap}
  struct PassabilityInfo {
    bool left_passable = false;   // 왼쪽으로 통과 가능
    bool right_passable = false;  // 오른쪽으로 통과 가능
    double left_gap = 0.0;        // 상대차량 왼쪽과 벽 사이 갭 (m)
    double right_gap = 0.0;       // 상대차량 오른쪽과 벽 사이 갭 (m)
  };
  PassabilityInfo checkPassability(double opp_x, double opp_y, double opp_yaw) const;

  // Late braking 경로 후보 생성
  struct LateBrakingCandidate {
    std::vector<double> path_x;
    std::vector<double> path_y;
    std::vector<double> v_desired;
    double outside_offset;
    double inside_offset;
    size_t brake_point_idx;
    size_t apex_idx;
    int corner_sign;
  };

  // apex 정보와 현재 상태로 late braking 후보 경로들 생성
  std::vector<LateBrakingCandidate> generateLateBrakingCandidates(
      const std::vector<int>& indices,
      const std::vector<double>& base_v,
      const std::vector<double>& base_kappa,
      const std::vector<double>& ds_from_start,
      double current_speed,
      size_t apex_idx, double apex_curvature, int corner_sign) const;

  LocalPlannerConfig config_;
  TrackBoundaryChecker checker_;
  std::shared_ptr<ReferenceLine> ref_;
  std::shared_ptr<OpponentPredictor> opp_pred_;

  int T_;  // number of timesteps in horizon

  // Hysteresis state
  double prev_lat_offset_ = 0.0;
  bool has_prev_offset_ = false;

  // Side commitment during overtake
  int committed_side_ = 0;  // -1: left, 0: none, +1: right
  bool was_overtaking_ = false;
};

}  // namespace scenario_director
