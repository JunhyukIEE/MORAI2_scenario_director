#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/overtake_strategy.hpp"
#include "scenario_director/pure_pursuit.hpp"
#include "scenario_director/slow_in_out.hpp"
#include "scenario_director/track_boundary.hpp"
#include "scenario_director/waypoint_selector.hpp"

#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>

namespace scenario_director {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string resolvePath(const std::string &path, const std::string &package_name) {
  if (path.empty()) {
    return path;
  }
  std::filesystem::path p(path);
  if (p.is_absolute()) {
    return path;
  }
  const std::string share = ament_index_cpp::get_package_share_directory(package_name);
  return (std::filesystem::path(share) / p).string();
}

}  // namespace

class ScenarioDirectorNode : public rclcpp::Node {
public:
  ScenarioDirectorNode()
  : rclcpp::Node("scenario_director") {
    declareParameters();
    loadParameters();

    initModules();
    setupRos();

    RCLCPP_INFO(get_logger(), "Scenario Director initialized");
  }

private:
  void declareParameters() {
    declare_parameter<std::string>("map.map_dir", "/home/ljh/Downloads/map");
    declare_parameter<std::string>("map.map_yaml", "Sangam_map.yaml");
    declare_parameter<std::string>("map.waypoints", "waypoints.csv");

    declare_parameter<std::string>("lines.optimal_path", "");
    declare_parameter<double>("lines.inside_offset", -1.5);
    declare_parameter<double>("lines.outside_offset", 1.5);

    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.width", 1.8);
    declare_parameter<double>("vehicle.max_steering", 0.55);
    declare_parameter<double>("vehicle.speed_multiplier", 1.15);

    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);

    declare_parameter<double>("selector.overtake_distance", 15.0);
    declare_parameter<double>("selector.overtake_angle", 30.0);
    declare_parameter<double>("selector.min_overtake_speed_diff", 3.0);
    declare_parameter<double>("selector.complete_distance", 10.0);
    declare_parameter<double>("selector.complete_angle", 150.0);
    declare_parameter<double>("selector.safe_distance", 20.0);
    declare_parameter<double>("selector.min_safe_gap", 5.0);
    declare_parameter<double>("selector.ttc_threshold", 2.0);
    declare_parameter<double>("selector.rear_danger_angle", 120.0);

    declare_parameter<bool>("strategy.slow_in_out.enabled", true);
    declare_parameter<double>("strategy.slow_in_out.apex_search_distance", 25.0);
    declare_parameter<double>("strategy.slow_in_out.entry_distance", 12.0);
    declare_parameter<double>("strategy.slow_in_out.exit_distance", 12.0);
    declare_parameter<double>("strategy.slow_in_out.entry_speed_scale", 0.85);
    declare_parameter<double>("strategy.slow_in_out.exit_speed_scale", 1.05);
    declare_parameter<double>("strategy.slow_in_out.curvature_threshold", 0.08);
    declare_parameter<bool>("strategy.slow_in_out.aggressive_exit", true);
    declare_parameter<double>("strategy.slow_in_out.overtake_exit_speed_scale", 1.15);
    declare_parameter<double>("strategy.slow_in_out.overtake_entry_speed_scale", 0.80);

    // Overtake Strategy parameters
    declare_parameter<double>("strategy.overtake.late_brake_distance", 8.0);
    declare_parameter<double>("strategy.overtake.late_brake_speed_scale", 0.75);
    declare_parameter<double>("strategy.overtake.block_pass_offset", 2.5);
    declare_parameter<double>("strategy.overtake.dummy_trigger_distance", 20.0);
    declare_parameter<double>("strategy.overtake.dummy_offset", 1.5);
    declare_parameter<double>("strategy.overtake.dummy_duration", 0.5);
    declare_parameter<double>("strategy.overtake.dummy_switch_threshold", 0.8);
    declare_parameter<double>("strategy.overtake.outside_entry_speed_scale", 0.9);
    declare_parameter<double>("strategy.overtake.outside_exit_boost", 1.15);
    declare_parameter<double>("strategy.overtake.chicane_detection_dist", 40.0);
    declare_parameter<double>("strategy.overtake.min_corner_curvature", 0.05);
    declare_parameter<double>("strategy.overtake.apex_search_distance", 50.0);

    declare_parameter<double>("pure_pursuit.lookahead_distance", 8.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 4.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 15.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.3);

    declare_parameter<int>("control.loop_hz", 20);
  }

  void loadParameters() {
    map_dir_ = get_parameter("map.map_dir").as_string();
    map_yaml_ = get_parameter("map.map_yaml").as_string();
    waypoints_file_ = get_parameter("map.waypoints").as_string();

    optimal_path_ = get_parameter("lines.optimal_path").as_string();
    inside_offset_ = get_parameter("lines.inside_offset").as_double();
    outside_offset_ = get_parameter("lines.outside_offset").as_double();

    wheelbase_ = get_parameter("vehicle.wheelbase").as_double();
    vehicle_width_ = get_parameter("vehicle.width").as_double();
    max_steering_ = get_parameter("vehicle.max_steering").as_double();
    speed_multiplier_ = get_parameter("vehicle.speed_multiplier").as_double();

    selector_config_.overtake_distance = get_parameter("selector.overtake_distance").as_double();
    selector_config_.overtake_angle = get_parameter("selector.overtake_angle").as_double();
    selector_config_.min_overtake_speed_diff = get_parameter("selector.min_overtake_speed_diff").as_double();
    selector_config_.complete_distance = get_parameter("selector.complete_distance").as_double();
    selector_config_.complete_angle = get_parameter("selector.complete_angle").as_double();
    selector_config_.safe_distance = get_parameter("selector.safe_distance").as_double();
    selector_config_.min_safe_gap = get_parameter("selector.min_safe_gap").as_double();
    selector_config_.ttc_threshold = get_parameter("selector.ttc_threshold").as_double();
    selector_config_.rear_danger_angle = get_parameter("selector.rear_danger_angle").as_double();
    selector_config_.vehicle_width = vehicle_width_;

    slow_in_out_config_.enabled = get_parameter("strategy.slow_in_out.enabled").as_bool();
    slow_in_out_config_.apex_search_distance =
        get_parameter("strategy.slow_in_out.apex_search_distance").as_double();
    slow_in_out_config_.entry_distance =
        get_parameter("strategy.slow_in_out.entry_distance").as_double();
    slow_in_out_config_.exit_distance =
        get_parameter("strategy.slow_in_out.exit_distance").as_double();
    slow_in_out_config_.entry_speed_scale =
        get_parameter("strategy.slow_in_out.entry_speed_scale").as_double();
    slow_in_out_config_.exit_speed_scale =
        get_parameter("strategy.slow_in_out.exit_speed_scale").as_double();
    slow_in_out_config_.curvature_threshold =
        get_parameter("strategy.slow_in_out.curvature_threshold").as_double();
    slow_in_out_config_.aggressive_exit =
        get_parameter("strategy.slow_in_out.aggressive_exit").as_bool();
    slow_in_out_config_.overtake_exit_speed_scale =
        get_parameter("strategy.slow_in_out.overtake_exit_speed_scale").as_double();
    slow_in_out_config_.overtake_entry_speed_scale =
        get_parameter("strategy.slow_in_out.overtake_entry_speed_scale").as_double();
    slow_in_out_planner_.setConfig(slow_in_out_config_);

    // Overtake Strategy config
    overtake_config_.late_brake_distance =
        get_parameter("strategy.overtake.late_brake_distance").as_double();
    overtake_config_.late_brake_speed_scale =
        get_parameter("strategy.overtake.late_brake_speed_scale").as_double();
    overtake_config_.block_pass_offset =
        get_parameter("strategy.overtake.block_pass_offset").as_double();
    overtake_config_.dummy_trigger_distance =
        get_parameter("strategy.overtake.dummy_trigger_distance").as_double();
    overtake_config_.dummy_offset =
        get_parameter("strategy.overtake.dummy_offset").as_double();
    overtake_config_.dummy_duration =
        get_parameter("strategy.overtake.dummy_duration").as_double();
    overtake_config_.dummy_switch_threshold =
        get_parameter("strategy.overtake.dummy_switch_threshold").as_double();
    overtake_config_.outside_entry_speed_scale =
        get_parameter("strategy.overtake.outside_entry_speed_scale").as_double();
    overtake_config_.outside_exit_boost =
        get_parameter("strategy.overtake.outside_exit_boost").as_double();
    overtake_config_.chicane_detection_dist =
        get_parameter("strategy.overtake.chicane_detection_dist").as_double();
    overtake_config_.min_corner_curvature =
        get_parameter("strategy.overtake.min_corner_curvature").as_double();
    overtake_config_.apex_search_distance =
        get_parameter("strategy.overtake.apex_search_distance").as_double();
    overtake_planner_.setConfig(overtake_config_);

    pp_config_.wheelbase = wheelbase_;
    pp_config_.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    pp_config_.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    pp_config_.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    pp_config_.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();
    pp_config_.max_steering = max_steering_;

    control_hz_ = get_parameter("control.loop_hz").as_int();
    throttle_kp_ = get_parameter("control.throttle_kp").as_double();
    brake_kp_ = get_parameter("control.brake_kp").as_double();
    max_throttle_ = get_parameter("control.max_throttle").as_double();
    max_brake_ = get_parameter("control.max_brake").as_double();
  }

  void initModules() {
    line_manager_ = std::make_shared<LineManager>();
    line_manager_->setSpeedMultiplier(speed_multiplier_);

    std::string optimal_path = optimal_path_;
    if (!optimal_path.empty()) {
      optimal_path = resolvePath(optimal_path, "scenario_director");
    } else {
      std::string map_dir = map_dir_;
      if (!map_dir.empty()) {
        if (!std::filesystem::path(map_dir).is_absolute()) {
          map_dir = resolvePath(map_dir, "scenario_director");
        }
        optimal_path = (std::filesystem::path(map_dir) / waypoints_file_).string();
      }
    }

    if (!optimal_path.empty()) {
      try {
        line_manager_->loadOptimalLine(optimal_path);
        line_manager_->generateOffsetLines(inside_offset_, outside_offset_);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to load lines: %s", e.what());
      }
    } else {
      RCLCPP_WARN(get_logger(), "No optimal path configured.");
    }

    if (!map_dir_.empty()) {
      std::string map_dir = map_dir_;
      if (!std::filesystem::path(map_dir).is_absolute()) {
        map_dir = resolvePath(map_dir, "scenario_director");
      }
      try {
        boundary_checker_ = std::make_shared<TrackBoundaryChecker>(map_dir, map_yaml_);
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Failed to init boundary checker: %s", e.what());
      }
    }

    selector_ = std::make_shared<WaypointSelector>(line_manager_, selector_config_, boundary_checker_);
    controller_ = std::make_shared<PurePursuitController>(pp_config_);

  }

  void setupRos() {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10,
      std::bind(&ScenarioDirectorNode::odomCallback, this, std::placeholders::_1));

    velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10,
      std::bind(&ScenarioDirectorNode::velocityCallback, this, std::placeholders::_1));

    steering_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/steering_state", 10,
      std::bind(&ScenarioDirectorNode::steeringCallback, this, std::placeholders::_1));

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opponent/odom", 10,
      std::bind(&ScenarioDirectorNode::oppOdomCallback, this, std::placeholders::_1));

    opp_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/opponent/vehicle/velocity", 10,
      std::bind(&ScenarioDirectorNode::oppVelocityCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ego/ctrl_cmd", 10);
    strategy_pub_ = create_publisher<std_msgs::msg::String>("/ego/overtake_strategy", 10);

    const double period = 1.0 / static_cast<double>(control_hz_ > 0 ? control_hz_ : 20);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&ScenarioDirectorNode::controlLoop, this));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!ego_state_) {
      ego_state_ = std::make_shared<VehicleState>();
    }
    ego_state_->x = msg->pose.pose.position.x;
    ego_state_->y = msg->pose.pose.position.y;
    ego_state_->yaw = quaternionToYaw(msg->pose.pose.orientation);
  }

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (ego_state_) {
      ego_state_->speed = msg->data;
    }
  }

  void steeringCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  }

  void oppOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!opp_state_) {
      opp_state_ = std::make_shared<VehicleState>();
    }
    opp_state_->x = msg->pose.pose.position.x;
    opp_state_->y = msg->pose.pose.position.y;
    opp_state_->yaw = quaternionToYaw(msg->pose.pose.orientation);
  }

  void oppVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (opp_state_) {
      opp_state_->speed = msg->data;
    }
  }

  void controlLoop() {
    if (!ego_state_) {
      return;
    }

    auto [line, line_name] = selector_->update(*ego_state_, opp_state_);
    if (!line) {
      return;
    }

    const DrivingState driving_state = selector_->getState();
    const bool is_overtaking = selector_->isOvertaking();

    // 추월 전략 적용
    std::string final_line_name = line_name;
    double strategy_speed_modifier = 1.0;
    OvertakeStrategyType active_strategy = OvertakeStrategyType::NONE;

    if (is_overtaking && opp_state_) {
      StrategyResult strategy = overtake_planner_.planStrategy(
          *line, *ego_state_, *opp_state_, driving_state, line_name);

      active_strategy = strategy.active_strategy;
      strategy_speed_modifier = strategy.speed_modifier;

      // 전략이 추천하는 라인으로 변경
      if (!strategy.recommended_line.empty() && strategy.recommended_line != line_name) {
        auto new_line = line_manager_->getLine(strategy.recommended_line);
        if (new_line) {
          line = new_line;
          final_line_name = strategy.recommended_line;
        }
      }
    }

    auto [steering, target_speed] = controller_->compute(
      ego_state_->x,
      ego_state_->y,
      ego_state_->yaw,
      ego_state_->speed,
      *line);

    // Slow In, Fast Out 적용 (추월 상태 전달)
    target_speed = slow_in_out_planner_.adjustTargetSpeed(*line, *ego_state_, target_speed, is_overtaking);

    // 전략에서 제공한 속도 배율 적용
    target_speed *= strategy_speed_modifier;

    // Compute throttle and brake from speed error
    const double speed_error = target_speed - ego_state_->speed;
    double throttle = 0.0;
    double brake = 0.0;

    if (speed_error > 0.0) {
      throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
    } else {
      brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error));
    }

    // Clamp steering
    steering = std::max(-max_steering_, std::min(max_steering_, steering));

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = throttle;
    cmd.linear.y = brake;
    cmd.angular.z = steering;
    cmd_pub_->publish(cmd);

    // 전략 상태 퍼블리시
    if (strategy_pub_) {
      std_msgs::msg::String strategy_msg;
      strategy_msg.data = OvertakeStrategyPlanner::strategyToString(active_strategy);
      strategy_pub_->publish(strategy_msg);
    }

    if (driving_state != DrivingState::NORMAL) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "State: %d, Strategy: %s, Line: %s, Speed: %.1f m/s, Modifier: %.2f",
                           static_cast<int>(driving_state),
                           OvertakeStrategyPlanner::strategyToString(active_strategy).c_str(),
                           final_line_name.c_str(), ego_state_->speed, strategy_speed_modifier);
    }
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<TrackBoundaryChecker> boundary_checker_;
  std::shared_ptr<WaypointSelector> selector_;
  std::shared_ptr<PurePursuitController> controller_;


  std::shared_ptr<VehicleState> ego_state_;
  std::shared_ptr<VehicleState> opp_state_;


  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr opp_velocity_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr strategy_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::string map_dir_;
  std::string map_yaml_;
  std::string waypoints_file_;
  std::string optimal_path_;

  double inside_offset_ = -1.5;
  double outside_offset_ = 1.5;
  double wheelbase_ = 2.7;
  double vehicle_width_ = 1.8;
  double max_steering_ = 0.55;
  double speed_multiplier_ = 1.15;

  int control_hz_ = 20;
  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;

  SelectorConfig selector_config_;
  PurePursuitConfig pp_config_;
  SlowInOutConfig slow_in_out_config_;
  SlowInOutPlanner slow_in_out_planner_;
  OvertakeStrategyConfig overtake_config_;
  OvertakeStrategyPlanner overtake_planner_;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::ScenarioDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
