#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/msg/vehicle_cmd.hpp"
#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"
#include "scenario_director/local_path_planner.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

namespace scenario_director {

namespace {

double quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

// Driving mode enumeration
enum class DrivingMode {
  WAYPOINT_FOLLOW,   // Normal waypoint following
  OVERTAKE_PLANNING, // Using LocalPathPlanner for overtaking
  OVERTAKE_EXECUTE   // Executing planned overtake trajectory
};

std::string drivingModeToString(DrivingMode mode) {
  switch (mode) {
    case DrivingMode::WAYPOINT_FOLLOW: return "WAYPOINT_FOLLOW";
    case DrivingMode::OVERTAKE_PLANNING: return "OVERTAKE_PLANNING";
    case DrivingMode::OVERTAKE_EXECUTE: return "OVERTAKE_EXECUTE";
    default: return "UNKNOWN";
  }
}

class RacingDirectorNode : public rclcpp::Node {
public:
  RacingDirectorNode() : rclcpp::Node("racing_director") {
    initializeParameters();
    initializePurePursuit();
    initializeLocalPlanner();
    initializeSubscribers();
    initializePublishers();
    initializeTimer();

    RCLCPP_INFO(get_logger(), "Racing Director initialized");
    RCLCPP_INFO(get_logger(), "  Overtake trigger distance: %.1f m", overtake_trigger_distance_);
    RCLCPP_INFO(get_logger(), "  Overtake complete margin: %.1f m", overtake_complete_margin_);
  }

private:
  void initializeParameters() {
    // Map and waypoint paths
    declare_parameter<std::string>("map.map_dir", "map");
    declare_parameter<std::string>("map.map_yaml", "Sangam_map.yaml");
    declare_parameter<std::string>("map.waypoints", "waypoints.csv");

    // Waypoint settings
    declare_parameter<double>("waypoints.default_speed", 5.0);

    // Vehicle parameters
    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.max_steering", 0.55);
    declare_parameter<double>("vehicle.max_steer_rad", 0.489);
    declare_parameter<double>("vehicle.max_accel", 3.5);
    declare_parameter<double>("vehicle.max_decel", 7.0);
    declare_parameter<double>("vehicle.max_lat_accel", 8.5);

    // Control parameters
    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);
    declare_parameter<double>("control.loop_hz", 20.0);

    // Pure pursuit parameters
    declare_parameter<double>("pure_pursuit.lookahead_distance", 8.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 4.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 15.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.3);
    declare_parameter<double>("pure_pursuit.steering_scale", 1.0);

    // Mode switching parameters
    declare_parameter<double>("overtake.trigger_distance", 30.0);
    declare_parameter<double>("overtake.complete_margin", 10.0);
    declare_parameter<double>("overtake.return_to_line_distance", 5.0);

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

    // Planning settings
    declare_parameter<double>("planner.dt", 0.1);
    declare_parameter<double>("planner.horizon_s", 4.0);
    declare_parameter<double>("planner.path_ahead_m", 70.0);
    declare_parameter<double>("planner.planning_rate_hz", 10.0);

    // Lateral offsets
    declare_parameter<std::vector<double>>("lateral_offsets",
        std::vector<double>{-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0});
    declare_parameter<std::vector<double>>("speed_scales",
        std::vector<double>{0.85, 1.0, 1.10, 1.20});

    // Load parameters
    throttle_kp_ = get_parameter("control.throttle_kp").as_double();
    brake_kp_ = get_parameter("control.brake_kp").as_double();
    max_throttle_ = get_parameter("control.max_throttle").as_double();
    max_brake_ = get_parameter("control.max_brake").as_double();
    max_steering_ = get_parameter("vehicle.max_steering").as_double();
    overtake_trigger_distance_ = get_parameter("overtake.trigger_distance").as_double();
    overtake_complete_margin_ = get_parameter("overtake.complete_margin").as_double();
    return_to_line_distance_ = get_parameter("overtake.return_to_line_distance").as_double();
  }

  void initializePurePursuit() {
    const std::string share = ament_index_cpp::get_package_share_directory("scenario_director");
    const std::string map_dir_rel = get_parameter("map.map_dir").as_string();
    const std::string waypoints_rel = get_parameter("map.waypoints").as_string();

    std::filesystem::path map_dir_path = map_dir_rel;
    if (!map_dir_path.is_absolute()) {
      map_dir_path = std::filesystem::path(share) / map_dir_rel;
    }
    std::filesystem::path waypoints_path = waypoints_rel;
    if (!waypoints_path.is_absolute()) {
      waypoints_path = map_dir_path / waypoints_rel;
    }

    const double default_speed = get_parameter("waypoints.default_speed").as_double();

    PurePursuitConfig cfg;
    cfg.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    cfg.max_steering = get_parameter("vehicle.max_steering").as_double();
    cfg.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    cfg.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    cfg.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    cfg.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();
    cfg.steering_scale = get_parameter("pure_pursuit.steering_scale").as_double();
    pp_config_ = cfg;

    pp_controller_ = std::make_shared<PurePursuitController>(cfg);

    line_manager_ = std::make_shared<LineManager>();
    line_manager_->setDefaultSpeed(default_speed);
    try {
      line_manager_->loadOptimalLine(waypoints_path.string());
      racing_line_ = line_manager_->getLine("optimal");
      RCLCPP_INFO(get_logger(), "Loaded waypoints: %zu points", racing_line_->size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints: %s", e.what());
    }
  }

  void initializeLocalPlanner() {
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

    // Build planner config
    LocalPlannerConfig config;

    config.vehicle.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    config.vehicle.max_steer = get_parameter("vehicle.max_steer_rad").as_double();
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

    config.dt = get_parameter("planner.dt").as_double();
    config.horizon_s = get_parameter("planner.horizon_s").as_double();
    config.path_ahead_m = get_parameter("planner.path_ahead_m").as_double();

    config.lateral_offsets = get_parameter("lateral_offsets").as_double_array();
    config.speed_scales = get_parameter("speed_scales").as_double_array();

    try {
      local_planner_ = std::make_shared<LocalPathPlanner>(
          map_dir_path.string(), map_yaml, waypoints_path.string(), config);
      RCLCPP_INFO(get_logger(), "LocalPathPlanner initialized");
      RCLCPP_INFO(get_logger(), "  Track length: %.1f m",
                  local_planner_->getReferenceLine()->trackLength());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize LocalPathPlanner: %s", e.what());
    }
  }

  void initializeSubscribers() {
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/ego/odom", 10,
        std::bind(&RacingDirectorNode::egoOdomCallback, this, std::placeholders::_1));

    ego_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/ego/vehicle/velocity", 10,
        std::bind(&RacingDirectorNode::egoVelocityCallback, this, std::placeholders::_1));

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/opponent/odom", 10,
        std::bind(&RacingDirectorNode::oppOdomCallback, this, std::placeholders::_1));

    opp_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/opponent/vehicle/velocity", 10,
        std::bind(&RacingDirectorNode::oppVelocityCallback, this, std::placeholders::_1));
  }

  void initializePublishers() {
    cmd_pub_ = create_publisher<scenario_director::msg::VehicleCmd>("/ego/ctrl_cmd", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/racing_director/mode", 10);
    overtake_flag_pub_ = create_publisher<std_msgs::msg::Bool>("/ego/overtake_flag", 10);
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("/racing_director/planned_path", 10);
  }

  void initializeTimer() {
    const double loop_hz = get_parameter("control.loop_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / loop_hz);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RacingDirectorNode::controlLoop, this));
  }

  // Callbacks
  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    ego_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    ego_odom_received_ = true;
  }

  void egoVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ego_speed_ = msg->data;
  }

  void oppOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    opp_x_ = msg->pose.pose.position.x;
    opp_y_ = msg->pose.pose.position.y;
    opp_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    opp_odom_received_ = true;
  }

  void oppVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    opp_speed_ = msg->data;
  }

  // Main control loop
  void controlLoop() {
    double ego_x, ego_y, ego_yaw, ego_speed;
    double opp_x, opp_y, opp_speed;
    bool has_ego, has_opp;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      ego_x = ego_x_;
      ego_y = ego_y_;
      ego_yaw = ego_yaw_;
      ego_speed = ego_speed_;
      opp_x = opp_x_;
      opp_y = opp_y_;
      opp_speed = opp_speed_;
      has_ego = ego_odom_received_;
      has_opp = opp_odom_received_;
    }

    if (!has_ego || !racing_line_) {
      return;
    }

    // Update driving mode based on opponent position
    updateDrivingMode(ego_x, ego_y, ego_yaw, opp_x, opp_y, has_opp);

    // Execute control based on current mode
    double steering = 0.0;
    double target_speed = 0.0;

    switch (current_mode_) {
      case DrivingMode::WAYPOINT_FOLLOW:
        executeWaypointFollow(ego_x, ego_y, ego_yaw, ego_speed, steering, target_speed);
        break;

      case DrivingMode::OVERTAKE_PLANNING:
      case DrivingMode::OVERTAKE_EXECUTE:
        executeOvertake(ego_x, ego_y, ego_yaw, ego_speed,
                       opp_x, opp_y, opp_speed, has_opp,
                       steering, target_speed);
        break;
    }

    // Compute throttle/brake from speed error
    const double speed_error = target_speed - ego_speed;
    double throttle = 0.0;
    double brake = 0.0;

    if (speed_error > 0.0) {
      throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
    } else {
      brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error));
    }

    steering = std::clamp(steering, -max_steering_, max_steering_);

    // Publish command
    scenario_director::msg::VehicleCmd cmd;
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.steering = steering;
    cmd_pub_->publish(cmd);

    // Publish mode status
    std_msgs::msg::String mode_msg;
    mode_msg.data = drivingModeToString(current_mode_);
    mode_pub_->publish(mode_msg);

    std_msgs::msg::Bool flag_msg;
    flag_msg.data = (current_mode_ != DrivingMode::WAYPOINT_FOLLOW);
    overtake_flag_pub_->publish(flag_msg);

    // Periodic logging
    static int log_counter = 0;
    if (++log_counter >= 100) {  // Every 5 seconds at 20 Hz
      log_counter = 0;
      RCLCPP_INFO(get_logger(), "Mode: %s | Ego: (%.1f, %.1f) v=%.1f | Opp dist: %.1f m",
                  drivingModeToString(current_mode_).c_str(),
                  ego_x, ego_y, ego_speed,
                  has_opp ? std::hypot(opp_x - ego_x, opp_y - ego_y) : -1.0);
    }
  }

  void updateDrivingMode(double ego_x, double ego_y, double ego_yaw,
                         double opp_x, double opp_y, bool has_opp) {
    if (!has_opp) {
      // No opponent data, stay in waypoint follow
      if (current_mode_ != DrivingMode::WAYPOINT_FOLLOW) {
        RCLCPP_INFO(get_logger(), "Returning to WAYPOINT_FOLLOW (no opponent)");
        current_mode_ = DrivingMode::WAYPOINT_FOLLOW;
      }
      return;
    }

    // Calculate relative position
    const double dx = opp_x - ego_x;
    const double dy = opp_y - ego_y;
    const double dist = std::hypot(dx, dy);

    // Check if opponent is ahead
    const double heading_x = std::cos(ego_yaw);
    const double heading_y = std::sin(ego_yaw);
    const double dot = dx * heading_x + dy * heading_y;
    const bool is_ahead = dot > 0.0;

    // Calculate s-coordinate difference using reference line
    double ego_s = 0.0, opp_s = 0.0;
    if (local_planner_) {
      auto ref = local_planner_->getReferenceLine();
      ego_s = ref->nearestS(ego_x, ego_y);
      opp_s = ref->nearestS(opp_x, opp_y);
    }

    // s-delta (positive if ego is ahead)
    double track_len = local_planner_ ? local_planner_->getReferenceLine()->trackLength() : 1e9;
    double s_delta = ego_s - opp_s;
    if (s_delta < -0.5 * track_len) s_delta += track_len;
    else if (s_delta > 0.5 * track_len) s_delta -= track_len;

    switch (current_mode_) {
      case DrivingMode::WAYPOINT_FOLLOW:
        // Trigger overtake if opponent is ahead and within trigger distance
        if (is_ahead && dist <= overtake_trigger_distance_) {
          RCLCPP_INFO(get_logger(),
              "Triggering OVERTAKE_PLANNING: opponent at %.1f m ahead", dist);
          current_mode_ = DrivingMode::OVERTAKE_PLANNING;
          overtake_start_time_ = now();
        }
        break;

      case DrivingMode::OVERTAKE_PLANNING:
        // Transition to execute after planning is ready
        if (planned_trajectory_.size() > 0) {
          current_mode_ = DrivingMode::OVERTAKE_EXECUTE;
          trajectory_index_ = 0;
          RCLCPP_INFO(get_logger(), "Transitioning to OVERTAKE_EXECUTE");
        }
        // Also check if overtake is complete (we somehow passed)
        if (s_delta > overtake_complete_margin_) {
          RCLCPP_INFO(get_logger(),
              "Overtake complete during planning (s_delta=%.1f m), returning to WAYPOINT_FOLLOW",
              s_delta);
          current_mode_ = DrivingMode::WAYPOINT_FOLLOW;
          planned_trajectory_.clear();
        }
        break;

      case DrivingMode::OVERTAKE_EXECUTE:
        // Check if overtake is complete
        if (s_delta > overtake_complete_margin_) {
          // We're ahead of opponent by enough margin
          RCLCPP_INFO(get_logger(),
              "Overtake COMPLETE (s_delta=%.1f m), returning to WAYPOINT_FOLLOW", s_delta);
          current_mode_ = DrivingMode::WAYPOINT_FOLLOW;
          planned_trajectory_.clear();
        }
        // Continue planning while executing
        break;
    }
  }

  void executeWaypointFollow(double ego_x, double ego_y, double ego_yaw, double ego_speed,
                             double& steering, double& target_speed) {
    const double lookahead = computeLookahead(ego_speed);
    Waypoint target = racing_line_->getLookaheadPoint(ego_x, ego_y, lookahead);

    steering = pp_controller_->computeSteeringForTarget(
        ego_x, ego_y, ego_yaw, target.x, target.y, lookahead);
    steering *= pp_config_.steering_scale;

    target_speed = target.speed;
  }

  void executeOvertake(double ego_x, double ego_y, double ego_yaw, double ego_speed,
                       double opp_x, double opp_y, double opp_speed, bool has_opp,
                       double& steering, double& target_speed) {
    if (!local_planner_ || !has_opp) {
      // Fallback to waypoint follow
      executeWaypointFollow(ego_x, ego_y, ego_yaw, ego_speed, steering, target_speed);
      return;
    }

    // Run planner to get new trajectory
    auto result = local_planner_->plan(ego_x, ego_y, ego_yaw, ego_speed,
                                        opp_x, opp_y, opp_speed);

    // Update planned trajectory
    planned_trajectory_ = result.trajectory;
    planned_v_desired_ = result.v_desired;

    // Publish planned path for visualization
    publishPlannedPath(result);

    if (planned_trajectory_.empty()) {
      // Fallback
      executeWaypointFollow(ego_x, ego_y, ego_yaw, ego_speed, steering, target_speed);
      return;
    }

    // Use first few points of trajectory for immediate control
    // Find closest point on trajectory
    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < planned_trajectory_.size(); ++i) {
      double dx = planned_trajectory_[i].x - ego_x;
      double dy = planned_trajectory_[i].y - ego_y;
      double d = dx * dx + dy * dy;
      if (d < min_dist) {
        min_dist = d;
        closest_idx = i;
      }
    }

    // Lookahead on planned trajectory
    const double lookahead = computeLookahead(ego_speed);
    double acc_dist = 0.0;
    size_t target_idx = closest_idx;

    for (size_t i = closest_idx; i + 1 < planned_trajectory_.size(); ++i) {
      double dx = planned_trajectory_[i + 1].x - planned_trajectory_[i].x;
      double dy = planned_trajectory_[i + 1].y - planned_trajectory_[i].y;
      acc_dist += std::hypot(dx, dy);
      if (acc_dist >= lookahead) {
        target_idx = i + 1;
        break;
      }
      target_idx = i + 1;
    }

    target_idx = std::min(target_idx, planned_trajectory_.size() - 1);

    double target_x = planned_trajectory_[target_idx].x;
    double target_y = planned_trajectory_[target_idx].y;

    // Pure pursuit steering towards trajectory point
    steering = pp_controller_->computeSteeringForTarget(
        ego_x, ego_y, ego_yaw, target_x, target_y, lookahead);
    steering *= pp_config_.steering_scale;

    // Use target speed from trajectory
    target_speed = planned_trajectory_[target_idx].v;

    // Log planning result periodically
    static int plan_log_counter = 0;
    if (++plan_log_counter >= 20) {  // Every 1 second at 20 Hz
      plan_log_counter = 0;
      RCLCPP_INFO(get_logger(),
          "Overtake plan: R=%.1f, lat=%.2f, sc=%.2f, target_v=%.1f",
          result.reward, result.lat_offset, result.speed_scale, target_speed);
    }
  }

  double computeLookahead(double speed) const {
    double lookahead = pp_config_.lookahead_distance + pp_config_.lookahead_ratio * speed;
    return std::clamp(lookahead, pp_config_.min_lookahead, pp_config_.max_lookahead);
  }

  void publishPlannedPath(const PlanResult& result) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = "map";

    for (const auto& pt : result.trajectory) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = pt.x;
      pose.pose.position.y = pt.y;
      pose.pose.position.z = 0.0;

      double cy = std::cos(pt.yaw * 0.5);
      double sy = std::sin(pt.yaw * 0.5);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = sy;
      pose.pose.orientation.w = cy;

      path_msg.poses.push_back(pose);
    }

    planned_path_pub_->publish(path_msg);
  }

  // Pure pursuit components
  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> racing_line_;
  std::shared_ptr<PurePursuitController> pp_controller_;
  PurePursuitConfig pp_config_;

  // Local path planner
  std::shared_ptr<LocalPathPlanner> local_planner_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr opp_vel_sub_;

  // Publishers
  rclcpp::Publisher<scenario_director::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtake_flag_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Control parameters
  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double max_steering_ = 0.55;

  // Mode switching parameters
  double overtake_trigger_distance_ = 30.0;
  double overtake_complete_margin_ = 10.0;
  double return_to_line_distance_ = 5.0;

  // State
  std::mutex data_mutex_;
  double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_speed_ = 0.0;
  double opp_x_ = 0.0, opp_y_ = 0.0, opp_yaw_ = 0.0, opp_speed_ = 0.0;
  bool ego_odom_received_ = false;
  bool opp_odom_received_ = false;

  // Driving mode state
  DrivingMode current_mode_ = DrivingMode::WAYPOINT_FOLLOW;
  rclcpp::Time overtake_start_time_;

  // Planned trajectory storage
  std::vector<TrajectoryPoint> planned_trajectory_;
  std::vector<double> planned_v_desired_;
  size_t trajectory_index_ = 0;
};

}  // namespace scenario_director

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::RacingDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
