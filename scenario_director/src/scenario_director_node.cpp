#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "scenario_director/msg/vehicle_cmd.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"

#include <cmath>
#include <filesystem>
#include <memory>
#include <vector>

namespace scenario_director {

namespace {

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
    declare_parameter<std::string>("map.waypoints", "map/waypoints.csv");
    declare_parameter<double>("waypoints.default_speed", 5.0);
    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.max_steering", 0.55);
    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);
    declare_parameter<double>("control.loop_hz", 20.0);
    declare_parameter<double>("pure_pursuit.lookahead_distance", 8.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 4.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 15.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.3);
    declare_parameter<double>("pure_pursuit.steering_scale", 1.0);

    const std::string waypoint_path = resolvePath(
        get_parameter("map.waypoints").as_string(), "scenario_director");
    const double default_speed = get_parameter("waypoints.default_speed").as_double();

    PurePursuitConfig cfg;
    cfg.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    cfg.max_steering = get_parameter("vehicle.max_steering").as_double();
    cfg.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    cfg.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    cfg.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    cfg.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();
    cfg.steering_scale = get_parameter("pure_pursuit.steering_scale").as_double();

    controller_ = std::make_shared<PurePursuitController>(cfg);

    line_manager_ = std::make_shared<LineManager>();
    line_manager_->setDefaultSpeed(default_speed);
    try {
      line_manager_->loadOptimalLine(waypoint_path);
      line_ = line_manager_->getLine("optimal");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load waypoints: %s", e.what());
    }

    throttle_kp_ = get_parameter("control.throttle_kp").as_double();
    brake_kp_ = get_parameter("control.brake_kp").as_double();
    max_throttle_ = get_parameter("control.max_throttle").as_double();
    max_brake_ = get_parameter("control.max_brake").as_double();
    max_steering_ = cfg.max_steering;
    pp_config_ = cfg;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10, std::bind(&ScenarioDirectorNode::odomCallback, this, std::placeholders::_1));
    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10, std::bind(&ScenarioDirectorNode::velocityCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<scenario_director::msg::VehicleCmd>("/ego/ctrl_cmd", 10);

    const double loop_hz = get_parameter("control.loop_hz").as_double();
    const int period_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, loop_hz)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&ScenarioDirectorNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Scenario Director (waypoint following mode) initialized");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    has_pose_ = true;
  }

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    speed_ = msg->data;
  }

  void controlLoop() {
    if (!has_pose_ || !line_) {
      return;
    }

    const double lookahead = computeLookahead();
    Waypoint target = line_->getLookaheadPoint(x_, y_, lookahead);

    double steering = controller_->computeSteeringForTarget(
        x_, y_, yaw_, target.x, target.y, lookahead);
    steering *= pp_config_.steering_scale;

    double target_speed = target.speed;

    const double speed_error = target_speed - speed_;
    double throttle = 0.0;
    double brake = 0.0;

    if (speed_error > 0.0) {
      throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
    } else {
      brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error));
    }

    steering = std::max(-max_steering_, std::min(max_steering_, steering));

    scenario_director::msg::VehicleCmd cmd;
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.steering = steering;
    cmd_pub_->publish(cmd);
  }

  double computeLookahead() const {
    double lookahead = pp_config_.lookahead_distance + pp_config_.lookahead_ratio * speed_;
    lookahead = std::clamp(lookahead, pp_config_.min_lookahead, pp_config_.max_lookahead);
    return lookahead;
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> line_;
  std::shared_ptr<PurePursuitController> controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
  rclcpp::Publisher<scenario_director::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double max_steering_ = 0.55;
  PurePursuitConfig pp_config_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double speed_ = 0.0;
  bool has_pose_ = false;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::ScenarioDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
