#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"
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

    declare_parameter<double>("selector.overtake_distance", 15.0);
    declare_parameter<double>("selector.overtake_angle", 30.0);
    declare_parameter<double>("selector.min_overtake_speed_diff", 3.0);
    declare_parameter<double>("selector.complete_distance", 10.0);
    declare_parameter<double>("selector.complete_angle", 150.0);
    declare_parameter<double>("selector.safe_distance", 20.0);
    declare_parameter<double>("selector.min_safe_gap", 5.0);
    declare_parameter<double>("selector.ttc_threshold", 2.0);
    declare_parameter<double>("selector.rear_danger_angle", 120.0);

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

    pp_config_.wheelbase = wheelbase_;
    pp_config_.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    pp_config_.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    pp_config_.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    pp_config_.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();
    pp_config_.max_steering = max_steering_;

    control_hz_ = get_parameter("control.loop_hz").as_int();
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

    auto [steering, target_speed] = controller_->compute(
      ego_state_->x,
      ego_state_->y,
      ego_state_->yaw,
      ego_state_->speed,
      *line);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed;
    cmd.angular.z = steering;
    cmd_pub_->publish(cmd);

    const auto state = selector_->getState();
    if (state != DrivingState::NORMAL) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "State: %d, Line: %s, Speed: %.1f m/s, Steering: %.1f deg",
                           static_cast<int>(state), line_name.c_str(), ego_state_->speed,
                           steering * kRadToDeg);
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

  SelectorConfig selector_config_;
  PurePursuitConfig pp_config_;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::ScenarioDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
