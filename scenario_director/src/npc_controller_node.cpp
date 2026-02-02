#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include "scenario_director/msg/vehicle_cmd.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"

#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>

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

class NPCControllerNode : public rclcpp::Node {
public:
  NPCControllerNode()
  : rclcpp::Node("npc_controller") {
    declare_parameter<std::string>("line.optimal_path", "map/waypoints.csv");
    declare_parameter<double>("npc.speed_ratio", 0.5);
    declare_parameter<double>("npc.min_speed_ratio", 0.2);
    declare_parameter<double>("npc.max_speed_ratio", 1.0);
    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.max_steering", 0.55);
    declare_parameter<double>("pure_pursuit.lookahead_distance", 6.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 3.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 10.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.2);
    declare_parameter<double>("pure_pursuit.steering_scale", 1.0);
    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);

    // Collision avoidance parameters
    declare_parameter<bool>("collision_avoidance.enabled", true);
    declare_parameter<double>("collision_avoidance.danger_distance", 5.0);
    declare_parameter<double>("collision_avoidance.caution_distance", 10.0);
    declare_parameter<double>("collision_avoidance.ttc_threshold", 2.0);
    declare_parameter<double>("collision_avoidance.side_margin", 2.0);
    declare_parameter<double>("collision_avoidance.min_speed_ratio", 0.2);
    declare_parameter<double>("collision_avoidance.brake_intensity", 0.8);

    const std::string line_path = resolvePath(get_parameter("line.optimal_path").as_string(),
                                              "scenario_director");
    speed_ratio_ = get_parameter("npc.speed_ratio").as_double();
    min_speed_ratio_npc_ = get_parameter("npc.min_speed_ratio").as_double();
    max_speed_ratio_ = get_parameter("npc.max_speed_ratio").as_double();

    // Clamp speed_ratio to valid range
    speed_ratio_ = std::max(min_speed_ratio_npc_, std::min(max_speed_ratio_, speed_ratio_));

    PurePursuitConfig cfg;
    cfg.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    cfg.max_steering = get_parameter("vehicle.max_steering").as_double();
    cfg.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    cfg.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    cfg.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    cfg.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();
    cfg.steering_scale = get_parameter("pure_pursuit.steering_scale").as_double();

    line_manager_ = std::make_shared<LineManager>();
    try {
      line_manager_->loadOptimalLine(line_path);
      line_ = line_manager_->getLine("optimal");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load line: %s", e.what());
    }

    controller_ = std::make_shared<PurePursuitController>(cfg);

    throttle_kp_ = get_parameter("control.throttle_kp").as_double();
    brake_kp_ = get_parameter("control.brake_kp").as_double();
    max_throttle_ = get_parameter("control.max_throttle").as_double();
    max_brake_ = get_parameter("control.max_brake").as_double();
    max_steering_ = cfg.max_steering;

    // Collision avoidance config
    collision_avoidance_enabled_ = get_parameter("collision_avoidance.enabled").as_bool();
    danger_distance_ = get_parameter("collision_avoidance.danger_distance").as_double();
    caution_distance_ = get_parameter("collision_avoidance.caution_distance").as_double();
    ttc_threshold_ = get_parameter("collision_avoidance.ttc_threshold").as_double();
    side_margin_ = get_parameter("collision_avoidance.side_margin").as_double();
    min_speed_ratio_ = get_parameter("collision_avoidance.min_speed_ratio").as_double();
    brake_intensity_ = get_parameter("collision_avoidance.brake_intensity").as_double();

    // NPC odom subscription
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opponent/odom", 10, std::bind(&NPCControllerNode::odomCallback, this, std::placeholders::_1));
    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/opponent/vehicle/velocity", 10, std::bind(&NPCControllerNode::velocityCallback, this, std::placeholders::_1));

    // Ego vehicle subscription for collision avoidance
    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10, std::bind(&NPCControllerNode::egoOdomCallback, this, std::placeholders::_1));
    ego_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10, std::bind(&NPCControllerNode::egoVelocityCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<scenario_director::msg::VehicleCmd>("/opponent/ctrl_cmd", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&NPCControllerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "NPC Controller initialized (collision avoidance: %s)",
                collision_avoidance_enabled_ ? "enabled" : "disabled");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!has_pose_) {
      has_pose_ = true;
    }
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  }

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    speed_ = msg->data;
  }

  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    ego_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    has_ego_pose_ = true;
  }

  void egoVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    ego_speed_ = msg->data;
  }

  struct CollisionRisk {
    bool is_dangerous = false;
    bool is_caution = false;
    double distance = 0.0;
    double ttc = std::numeric_limits<double>::infinity();
    double speed_modifier = 1.0;
    double brake_amount = 0.0;
  };

  CollisionRisk evaluateCollisionRisk() {
    CollisionRisk risk;

    if (!collision_avoidance_enabled_ || !has_ego_pose_) {
      return risk;
    }

    // Calculate relative position
    const double dx = ego_x_ - x_;
    const double dy = ego_y_ - y_;
    const double distance = std::sqrt(dx * dx + dy * dy);
    risk.distance = distance;

    // Calculate relative angle (ego relative to NPC's heading)
    const double abs_angle = std::atan2(dy, dx);
    double rel_angle = abs_angle - yaw_;
    rel_angle = std::atan2(std::sin(rel_angle), std::cos(rel_angle));

    // Check if ego is in front of NPC (within ±60 degrees)
    const bool ego_in_front = std::abs(rel_angle) < (60.0 * M_PI / 180.0);

    // Calculate lateral distance (perpendicular to NPC's heading)
    const double lateral_dist = std::abs(dx * (-std::sin(yaw_)) + dy * std::cos(yaw_));

    // Check if ego is on the same lane (within side margin)
    const bool same_lane = lateral_dist < side_margin_;

    if (!ego_in_front || !same_lane) {
      return risk;  // No risk if ego is behind or on different lane
    }

    // Calculate Time To Collision (TTC)
    const double closing_speed = speed_ - ego_speed_;
    if (closing_speed > 0.1) {
      risk.ttc = distance / closing_speed;
    }

    // Evaluate danger level
    if (distance < danger_distance_ || risk.ttc < ttc_threshold_) {
      risk.is_dangerous = true;
      risk.is_caution = true;

      // Emergency braking
      risk.speed_modifier = min_speed_ratio_;
      risk.brake_amount = brake_intensity_;

    } else if (distance < caution_distance_) {
      risk.is_caution = true;

      // Gradual speed reduction based on distance
      const double ratio = (distance - danger_distance_) / (caution_distance_ - danger_distance_);
      risk.speed_modifier = min_speed_ratio_ + (1.0 - min_speed_ratio_) * ratio;
      risk.brake_amount = brake_intensity_ * (1.0 - ratio) * 0.5;
    }

    return risk;
  }

  void controlLoop() {
    if (!has_pose_ || !line_) {
      return;
    }

    auto [steering, target_speed] = controller_->compute(x_, y_, yaw_, speed_, *line_);
    target_speed *= speed_ratio_;

    // Evaluate collision risk with ego vehicle
    const CollisionRisk risk = evaluateCollisionRisk();

    // Apply collision avoidance
    double throttle = 0.0;
    double brake = 0.0;

    if (risk.is_dangerous) {
      // Emergency: strong braking
      target_speed *= risk.speed_modifier;
      brake = std::min(max_brake_, risk.brake_amount);
      throttle = 0.0;

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                           "DANGER! Ego dist: %.1fm, TTC: %.1fs, braking!",
                           risk.distance, risk.ttc);

    } else if (risk.is_caution) {
      // Caution: gradual speed reduction
      target_speed *= risk.speed_modifier;

      const double speed_error = target_speed - speed_;
      if (speed_error > 0.0) {
        throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
      } else {
        brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error) + risk.brake_amount);
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Caution: Ego dist: %.1fm, speed modifier: %.2f",
                           risk.distance, risk.speed_modifier);

    } else {
      // Normal driving
      const double speed_error = target_speed - speed_;
      if (speed_error > 0.0) {
        throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
      } else {
        brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error));
      }
    }

    // Clamp steering
    steering = std::max(-max_steering_, std::min(max_steering_, steering));

    scenario_director::msg::VehicleCmd cmd;
    cmd.throttle = throttle;
    cmd.brake = brake;
    cmd.steering = steering;
    cmd_pub_->publish(cmd);
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> line_;
  std::shared_ptr<PurePursuitController> controller_;

  // NPC subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;

  // Ego subscriptions for collision avoidance
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_vel_sub_;

  rclcpp::Publisher<scenario_director::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // NPC state
  double speed_ratio_ = 0.5;
  double min_speed_ratio_npc_ = 0.2;
  double max_speed_ratio_ = 1.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double speed_ = 0.0;
  bool has_pose_ = false;

  // Ego state for collision avoidance
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_yaw_ = 0.0;
  double ego_speed_ = 0.0;
  bool has_ego_pose_ = false;

  // Control parameters
  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double max_steering_ = 0.55;

  // Collision avoidance parameters
  bool collision_avoidance_enabled_ = true;
  double danger_distance_ = 5.0;
  double caution_distance_ = 10.0;
  double ttc_threshold_ = 2.0;
  double side_margin_ = 2.0;
  double min_speed_ratio_ = 0.2;
  double brake_intensity_ = 0.8;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::NPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
