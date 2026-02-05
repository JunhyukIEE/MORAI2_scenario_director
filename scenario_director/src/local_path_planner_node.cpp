#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/local_path_planner.hpp"

#include <array>
#include <filesystem>
#include <mutex>

namespace scenario_director {

namespace {
constexpr int NUM_NPCS = 9;

struct NPCState {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  bool received = false;
};
}  // namespace

class LocalPathPlannerNode : public rclcpp::Node {
public:
  LocalPathPlannerNode() : rclcpp::Node("local_path_planner") {
    // Declare parameters
    declare_parameter<std::string>("map.map_dir", "map");
    declare_parameter<std::string>("map.map_yaml", "Sangam_map.yaml");
    declare_parameter<std::string>("map.waypoints", "waypoints.csv");

    // Vehicle parameters
    declare_parameter<double>("vehicle.wheelbase", 2.35);
    declare_parameter<double>("vehicle.max_steer", 0.436);
    declare_parameter<double>("vehicle.max_accel", 3.5);
    declare_parameter<double>("vehicle.max_decel", 7.0);
    declare_parameter<double>("vehicle.max_lat_accel", 8.5);

    // Planner weights
    declare_parameter<double>("weights.w_progress", 8.0);
    declare_parameter<double>("weights.w_speed", 1.5);
    declare_parameter<double>("weights.w_overtake", 60.0);
    declare_parameter<double>("weights.w_collision", 20000.0);
    declare_parameter<double>("weights.w_offroad", 4000.0);
    declare_parameter<double>("weights.w_smooth", 8.0);
    declare_parameter<double>("weights.w_curv", 2.0);
    declare_parameter<double>("weights.w_far_from_ref", 2.0);

    // Dimensions
    declare_parameter<double>("ego_length", 4.0);
    declare_parameter<double>("ego_width", 1.7);
    declare_parameter<double>("opp_length", 4.0);
    declare_parameter<double>("opp_width", 1.7);

    // 갭 통과 안전 설정
    declare_parameter<double>("pass_gap_margin", 0.8);
    declare_parameter<double>("gap_check_distance", 30.0);
    declare_parameter<double>("impassable_penalty", 50000.0);

    // Planning settings
    declare_parameter<double>("dt", 0.1);
    declare_parameter<double>("horizon_s", 4.0);
    declare_parameter<double>("path_ahead_m", 70.0);
    declare_parameter<double>("planning_rate_hz", 10.0);

    // Lateral offsets
    declare_parameter<std::vector<double>>("lateral_offsets",
        std::vector<double>{-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0});
    declare_parameter<std::vector<double>>("speed_scales",
        std::vector<double>{0.85, 1.0, 1.10, 1.20});

    // Safety
    declare_parameter<int>("offroad_check_stride", 2);
    declare_parameter<double>("overtake_margin_s", 6.0);
    declare_parameter<double>("near_pass_bonus_dist", 3.0);
    declare_parameter<double>("lat_change_penalty", 15.0);
    declare_parameter<double>("overtake_side_commit_penalty", 3000.0);

    // Late Braking (브레이킹 포인트 기반)
    declare_parameter<bool>("strategy.overtake.late_braking.enabled", true);
    declare_parameter<double>("strategy.overtake.late_braking.delay_factor", 0.3);
    declare_parameter<double>("strategy.overtake.late_braking.outside_offset", 2.5);
    declare_parameter<double>("strategy.overtake.late_braking.inside_offset", 1.5);
    declare_parameter<double>("strategy.overtake.late_braking.transition_sharpness", 2.0);
    declare_parameter<double>("strategy.overtake.late_braking.min_corner_curvature", 0.03);
    declare_parameter<double>("strategy.overtake.late_braking.safety_margin", 1.2);
    declare_parameter<double>("strategy.overtake.late_braking.max_decel_override", 0.0);
    declare_parameter<double>("strategy.overtake.late_braking.exit_recovery_distance", 15.0);

    // Overtake strategies
    declare_parameter<double>("strategy.overtake.min_corner_curvature", 0.05);
    declare_parameter<double>("strategy.overtake.apex_search_distance", 50.0);
    declare_parameter<double>("strategy.overtake.approach_speed_boost", 1.05);
    declare_parameter<double>("strategy.overtake.position_lateral_offset", 2.0);
    declare_parameter<double>("strategy.overtake.execute_speed_boost", 1.10);
    declare_parameter<double>("strategy.overtake.slipstream_distance", 8.0);
    declare_parameter<double>("strategy.overtake.slipstream_speed_boost", 1.08);
    declare_parameter<double>("strategy.overtake.gap_reward_scale", 200.0);

    // Load paths
    const std::string share = ament_index_cpp::get_package_share_directory("scenario_director");
    const std::string map_dir_rel = get_parameter("map.map_dir").as_string();
    const std::string map_yaml = get_parameter("map.map_yaml").as_string();
    const std::string waypoints_rel = get_parameter("map.waypoints").as_string();

    std::filesystem::path map_dir_path = map_dir_rel;
    if (!map_dir_path.is_absolute()) {
      map_dir_path = std::filesystem::path(share) / map_dir_rel;
    }
    std::filesystem::path waypoints_path = waypoints_rel;
    if (!waypoints_path.is_absolute()) {
      waypoints_path = map_dir_path / waypoints_rel;
    }

    // Build config
    LocalPlannerConfig config;

    config.vehicle.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    config.vehicle.max_steer = get_parameter("vehicle.max_steer").as_double();
    config.vehicle.max_accel = get_parameter("vehicle.max_accel").as_double();
    config.vehicle.max_decel = get_parameter("vehicle.max_decel").as_double();
    config.vehicle.max_lat_accel = get_parameter("vehicle.max_lat_accel").as_double();

    config.weights.w_progress = get_parameter("weights.w_progress").as_double();
    config.weights.w_speed = get_parameter("weights.w_speed").as_double();
    config.weights.w_overtake = get_parameter("weights.w_overtake").as_double();
    config.weights.w_collision = get_parameter("weights.w_collision").as_double();
    config.weights.w_offroad = get_parameter("weights.w_offroad").as_double();
    config.weights.w_smooth = get_parameter("weights.w_smooth").as_double();
    config.weights.w_curv = get_parameter("weights.w_curv").as_double();
    config.weights.w_far_from_ref = get_parameter("weights.w_far_from_ref").as_double();

    config.ego_length = get_parameter("ego_length").as_double();
    config.ego_width = get_parameter("ego_width").as_double();
    config.opp_length = get_parameter("opp_length").as_double();
    config.opp_width = get_parameter("opp_width").as_double();
    config.pass_gap_margin = get_parameter("pass_gap_margin").as_double();
    config.gap_check_distance = get_parameter("gap_check_distance").as_double();
    config.impassable_penalty = get_parameter("impassable_penalty").as_double();

    config.dt = get_parameter("dt").as_double();
    config.horizon_s = get_parameter("horizon_s").as_double();
    config.path_ahead_m = get_parameter("path_ahead_m").as_double();

    config.lateral_offsets = get_parameter("lateral_offsets").as_double_array();
    config.speed_scales = get_parameter("speed_scales").as_double_array();

    config.offroad_check_stride = get_parameter("offroad_check_stride").as_int();
    config.overtake_margin_s = get_parameter("overtake_margin_s").as_double();
    config.near_pass_bonus_dist = get_parameter("near_pass_bonus_dist").as_double();
    config.lat_change_penalty = get_parameter("lat_change_penalty").as_double();
    config.overtake_side_commit_penalty = get_parameter("overtake_side_commit_penalty").as_double();

    // Late Braking config
    config.strategy.overtake.late_braking.enabled =
        get_parameter("strategy.overtake.late_braking.enabled").as_bool();
    config.strategy.overtake.late_braking.delay_factor =
        get_parameter("strategy.overtake.late_braking.delay_factor").as_double();
    config.strategy.overtake.late_braking.outside_offset =
        get_parameter("strategy.overtake.late_braking.outside_offset").as_double();
    config.strategy.overtake.late_braking.inside_offset =
        get_parameter("strategy.overtake.late_braking.inside_offset").as_double();
    config.strategy.overtake.late_braking.transition_sharpness =
        get_parameter("strategy.overtake.late_braking.transition_sharpness").as_double();
    config.strategy.overtake.late_braking.min_corner_curvature =
        get_parameter("strategy.overtake.late_braking.min_corner_curvature").as_double();
    config.strategy.overtake.late_braking.safety_margin =
        get_parameter("strategy.overtake.late_braking.safety_margin").as_double();
    config.strategy.overtake.late_braking.max_decel_override =
        get_parameter("strategy.overtake.late_braking.max_decel_override").as_double();
    config.strategy.overtake.late_braking.exit_recovery_distance =
        get_parameter("strategy.overtake.late_braking.exit_recovery_distance").as_double();

    config.strategy.overtake.min_corner_curvature =
        get_parameter("strategy.overtake.min_corner_curvature").as_double();
    config.strategy.overtake.apex_search_distance =
        get_parameter("strategy.overtake.apex_search_distance").as_double();
    config.strategy.overtake.approach_speed_boost =
        get_parameter("strategy.overtake.approach_speed_boost").as_double();
    config.strategy.overtake.position_lateral_offset =
        get_parameter("strategy.overtake.position_lateral_offset").as_double();
    config.strategy.overtake.execute_speed_boost =
        get_parameter("strategy.overtake.execute_speed_boost").as_double();
    config.strategy.overtake.slipstream_distance =
        get_parameter("strategy.overtake.slipstream_distance").as_double();
    config.strategy.overtake.slipstream_speed_boost =
        get_parameter("strategy.overtake.slipstream_speed_boost").as_double();
    config.strategy.overtake.gap_reward_scale =
        get_parameter("strategy.overtake.gap_reward_scale").as_double();

    // Initialize planner
    planner_ = std::make_shared<LocalPathPlanner>(
        map_dir_path.string(), map_yaml, waypoints_path.string(), config);

    RCLCPP_INFO(get_logger(), "LocalPathPlanner initialized");
    RCLCPP_INFO(get_logger(), "  Track length: %.1f m", planner_->getReferenceLine()->trackLength());
    RCLCPP_INFO(get_logger(), "  Waypoints: %zu", planner_->getReferenceLine()->size());

    // Subscribers
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ego/odom", 10,
        std::bind(&LocalPathPlannerNode::egoOdomCallback, this, std::placeholders::_1));

    // NPC subscriptions (9 NPCs)
    for (int i = 0; i < NUM_NPCS; ++i) {
      std::string prefix = "/NPC_" + std::to_string(i + 1);
      npc_odom_subs_[i] = create_subscription<nav_msgs::msg::Odometry>(
        prefix + "/odom", 10,
        [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
          npcOdomCallback(msg, i);
        });
    }

    overtake_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/ego/overtake_flag", 10,
        std::bind(&LocalPathPlannerNode::overtakeFlagCallback, this, std::placeholders::_1));

    overtake_phase_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/ego/overtake_phase", 10,
        std::bind(&LocalPathPlannerNode::overtakePhaseCallback, this, std::placeholders::_1));

    target_npc_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/ego/target_npc_id", 10,
        std::bind(&LocalPathPlannerNode::targetNpcCallback, this, std::placeholders::_1));

    // Publishers
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planner/path", 10);
    path_speed_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/local_planner/path_speeds", 10);
    predicted_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planner/predicted_path", 10);
    candidate_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planner/candidate_path", 10);
    all_candidates_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/local_planner/all_candidates", 10);
    reward_pub_ = create_publisher<std_msgs::msg::Float64>("/local_planner/reward", 10);
    lat_offset_pub_ = create_publisher<std_msgs::msg::Float64>("/local_planner/lat_offset", 10);

    // Timer for planning loop
    double rate_hz = get_parameter("planning_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LocalPathPlannerNode::planningCallback, this));

    RCLCPP_INFO(get_logger(), "Planning loop started at %.1f Hz", rate_hz);
  }

private:
  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;

    // Extract yaw from quaternion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    ego_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    ego_v_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    ego_received_ = true;
  }

  void npcOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int npc_index) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    npc_states_[npc_index].x = msg->pose.pose.position.x;
    npc_states_[npc_index].y = msg->pose.pose.position.y;
    npc_states_[npc_index].v = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    npc_states_[npc_index].received = true;
  }

  void targetNpcCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_npc_id_ = msg->data - 1;  // Convert from 1-based to 0-based
  }

  void overtakeFlagCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    overtake_flag_ = msg->data;
  }

  void overtakePhaseCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    overtake_phase_ = msg->data;
  }

  void planningCallback() {
    double ego_x, ego_y, ego_yaw, ego_v;
    double opp_x, opp_y, opp_v;
    bool overtake_flag;
    int overtake_phase;
    bool can_plan = false;
    int used_npc_id = -1;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!ego_received_) {
        return;
      }

      ego_x = ego_x_;
      ego_y = ego_y_;
      ego_yaw = ego_yaw_;
      ego_v = ego_v_;
      overtake_flag = overtake_flag_;
      overtake_phase = overtake_phase_;

      // Use target NPC if available, otherwise find nearest NPC ahead
      int npc_to_use = target_npc_id_;
      if (npc_to_use < 0 || npc_to_use >= NUM_NPCS || !npc_states_[npc_to_use].received) {
        // Find nearest NPC ahead
        double nearest_dist = std::numeric_limits<double>::max();
        const double heading_x = std::cos(ego_yaw_);
        const double heading_y = std::sin(ego_yaw_);

        for (int i = 0; i < NUM_NPCS; ++i) {
          if (!npc_states_[i].received) continue;
          const double dx = npc_states_[i].x - ego_x_;
          const double dy = npc_states_[i].y - ego_y_;
          const double forward_dist = dx * heading_x + dy * heading_y;
          if (forward_dist > 0.0 && forward_dist < nearest_dist) {
            nearest_dist = forward_dist;
            npc_to_use = i;
          }
        }
      }

      if (npc_to_use >= 0 && npc_to_use < NUM_NPCS && npc_states_[npc_to_use].received) {
        opp_x = npc_states_[npc_to_use].x;
        opp_y = npc_states_[npc_to_use].y;
        opp_v = npc_states_[npc_to_use].v;
        used_npc_id = npc_to_use;
        can_plan = true;
      } else {
        // No NPC available, use dummy position far away
        opp_x = ego_x_ + 1000.0;
        opp_y = ego_y_;
        opp_v = 0.0;
        can_plan = true;
      }
    }

    if (!can_plan) {
      return;
    }

    // Run planner
    auto result = planner_->plan(ego_x, ego_y, ego_yaw, ego_v, opp_x, opp_y, opp_v, overtake_flag, overtake_phase);

    // Publish candidate path as the planned path (no curling at end)
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (size_t i = 0; i < result.candidate_path.size(); ++i) {
      const auto& [px, py] = result.candidate_path[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = px;
      pose.pose.position.y = py;
      pose.pose.position.z = 0.0;

      // Compute yaw from consecutive points
      double yaw = 0.0;
      if (i + 1 < result.candidate_path.size()) {
        double dx = result.candidate_path[i + 1].first - px;
        double dy = result.candidate_path[i + 1].second - py;
        yaw = std::atan2(dy, dx);
      } else if (i > 0) {
        double dx = px - result.candidate_path[i - 1].first;
        double dy = py - result.candidate_path[i - 1].second;
        yaw = std::atan2(dy, dx);
      }

      double cy = std::cos(yaw * 0.5);
      double sy = std::sin(yaw * 0.5);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sy;
      pose.pose.orientation.w = cy;

      path_msg.poses.push_back(pose);
    }
    planned_path_pub_->publish(path_msg);

    // Publish speed profile aligned with planned path
    std_msgs::msg::Float64MultiArray speeds_msg;
    speeds_msg.data = result.v_desired;
    path_speed_pub_->publish(speeds_msg);

    // Publish predicted trajectory path
    nav_msgs::msg::Path pred_msg;
    pred_msg.header.stamp = path_msg.header.stamp;
    pred_msg.header.frame_id = "map";
    for (const auto& pt : result.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = pred_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;

      double cy = std::cos(pt.yaw * 0.5);
      double sy = std::sin(pt.yaw * 0.5);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sy;
      pose.pose.orientation.w = cy;
      pred_msg.poses.push_back(pose);
    }
    predicted_path_pub_->publish(pred_msg);

    // Publish candidate path
    nav_msgs::msg::Path cand_msg;
    cand_msg.header = path_msg.header;
    for (const auto& [cx, cy] : result.candidate_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = cand_msg.header;
      pose.pose.position.x = cx;
      pose.pose.position.y = cy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      cand_msg.poses.push_back(pose);
    }
    candidate_path_pub_->publish(cand_msg);

    // Publish all candidate paths as MarkerArray (시각화용)
    visualization_msgs::msg::MarkerArray markers;

    // 먼저 이전 마커들 삭제
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = now();
    delete_marker.header.frame_id = "map";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    // reward 범위 계산 (색상 매핑용)
    double min_reward = std::numeric_limits<double>::max();
    double max_reward = std::numeric_limits<double>::lowest();
    for (const auto& cand : result.all_candidates) {
      min_reward = std::min(min_reward, cand.reward);
      max_reward = std::max(max_reward, cand.reward);
    }
    double reward_range = std::max(max_reward - min_reward, 1.0);

    int marker_id = 0;
    for (const auto& cand : result.all_candidates) {
      if (cand.path.empty()) continue;

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = "map";
      marker.ns = cand.is_late_braking ? "late_braking" : "normal";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.15;  // 선 두께

      // 색상: reward에 따라 빨강(낮음) → 초록(높음), Late Braking은 파랑 계열
      double t = (cand.reward - min_reward) / reward_range;
      if (cand.is_late_braking) {
        // Late Braking: 파랑 ~ 청록
        marker.color.r = 0.0;
        marker.color.g = t * 0.8;
        marker.color.b = 0.8 + t * 0.2;
        marker.color.a = 0.6;
      } else {
        // 일반 경로: 빨강 → 노랑 → 초록
        if (t < 0.5) {
          marker.color.r = 1.0;
          marker.color.g = t * 2.0;
          marker.color.b = 0.0;
        } else {
          marker.color.r = 1.0 - (t - 0.5) * 2.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
        }
        marker.color.a = 0.4;
      }

      // 포인트 추가 (간격 조절)
      for (size_t i = 0; i < cand.path.size(); i += 5) {
        geometry_msgs::msg::Point pt;
        pt.x = cand.path[i].first;
        pt.y = cand.path[i].second;
        pt.z = 0.1;
        marker.points.push_back(pt);
      }

      markers.markers.push_back(marker);
    }

    all_candidates_pub_->publish(markers);

    // Publish reward
    std_msgs::msg::Float64 reward_msg;
    reward_msg.data = result.reward;
    reward_pub_->publish(reward_msg);

    // Publish lateral offset
    std_msgs::msg::Float64 lat_msg;
    lat_msg.data = result.lat_offset;
    lat_offset_pub_->publish(lat_msg);

    // Log debug info periodically
    static int log_counter = 0;
    if (++log_counter >= 50) {  // Every 5 seconds at 10 Hz
      log_counter = 0;
      RCLCPP_INFO(get_logger(),
          "Plan: R=%.1f, lat=%.2f, sc=%.2f | Ego: (%.1f, %.1f) v=%.1f | Opp: (%.1f, %.1f) v=%.1f",
          result.reward, result.lat_offset, result.speed_scale,
          ego_x, ego_y, ego_v, opp_x, opp_y, opp_v);
    }
  }

  std::shared_ptr<LocalPathPlanner> planner_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  std::array<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr, NUM_NPCS> npc_odom_subs_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr overtake_flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr overtake_phase_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_npc_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_speed_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr all_candidates_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reward_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lat_offset_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Data mutex and state
  std::mutex data_mutex_;
  double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_v_ = 0.0;
  std::array<NPCState, NUM_NPCS> npc_states_;
  int target_npc_id_ = -1;  // 0-based index, -1 if none
  bool ego_received_ = false;
  bool overtake_flag_ = false;
  int overtake_phase_ = 0;  // 0=NONE, 1=APPROACH, 2=POSITION, 3=EXECUTE, 4=COMPLETE
};

}  // namespace scenario_director

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::LocalPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
