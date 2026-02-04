#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "scenario_director/msg/vehicle_cmd.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
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
    declare_parameter<double>("vehicle.speed_multiplier", 1.0);
    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);
    declare_parameter<double>("control.loop_hz", 20.0);
    declare_parameter<double>("pure_pursuit.lookahead_distance", 8.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 4.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 15.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.3);
    declare_parameter<double>("overtake.trigger_distance", 20.0);
    declare_parameter<double>("overtake.complete_distance", 8.0);
    declare_parameter<std::string>("local_path.topic", "/local_planner/path");
    declare_parameter<std::string>("local_speed.topic", "/local_planner/path_speeds");

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

    controller_ = std::make_shared<PurePursuitController>(cfg);

    const double speed_multiplier = get_parameter("vehicle.speed_multiplier").as_double();

    line_manager_ = std::make_shared<LineManager>();
    line_manager_->setDefaultSpeed(default_speed);
    line_manager_->setSpeedMultiplier(speed_multiplier);
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
    overtake_trigger_distance_ = get_parameter("overtake.trigger_distance").as_double();
    overtake_complete_distance_ = get_parameter("overtake.complete_distance").as_double();
    pp_config_ = cfg;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10, std::bind(&ScenarioDirectorNode::odomCallback, this, std::placeholders::_1));
    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10, std::bind(&ScenarioDirectorNode::velocityCallback, this, std::placeholders::_1));

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opponent/odom", 10, std::bind(&ScenarioDirectorNode::oppOdomCallback, this, std::placeholders::_1));
    opp_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/opponent/vehicle/velocity", 10, std::bind(&ScenarioDirectorNode::oppVelocityCallback, this, std::placeholders::_1));

    const std::string local_path_topic = get_parameter("local_path.topic").as_string();
    const std::string local_speed_topic = get_parameter("local_speed.topic").as_string();
    local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_path_topic, 10, std::bind(&ScenarioDirectorNode::localPathCallback, this, std::placeholders::_1));
    local_speed_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      local_speed_topic, 10, std::bind(&ScenarioDirectorNode::localSpeedCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<scenario_director::msg::VehicleCmd>("/ego/ctrl_cmd", 10);
    overtake_pub_ = create_publisher<std_msgs::msg::Bool>("/ego/overtake_flag", 10);

    const double loop_hz = get_parameter("control.loop_hz").as_double();
    const int period_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, loop_hz)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&ScenarioDirectorNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Scenario Director initialized (path: %s, speeds: %s)",
                local_path_topic.c_str(), local_speed_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    if (!msg->header.frame_id.empty()) {
      last_frame_id_ = msg->header.frame_id;
    }
    has_pose_ = true;
  }

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    speed_ = msg->data;
  }

  void oppOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    opp_x_ = msg->pose.pose.position.x;
    opp_y_ = msg->pose.pose.position.y;
    has_opp_pose_ = true;
  }

  void oppVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    opp_speed_ = msg->data;
  }

  void localPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(local_path_mutex_);
    local_path_.clear();
    for (const auto &pose : msg->poses) {
      Waypoint wp;
      wp.x = pose.pose.position.x;
      wp.y = pose.pose.position.y;
      wp.yaw = quaternionToYaw(pose.pose.orientation);
      wp.speed = 0.0;  // Speed will be determined by waypoints
      local_path_.push_back(wp);
    }
    has_local_path_ = !local_path_.empty();
  }

  void localSpeedCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(local_path_mutex_);
    local_path_speeds_ = msg->data;
    has_local_speeds_ = !local_path_speeds_.empty();
  }

  void controlLoop() {
    if (!has_pose_ || !line_) {
      return;
    }

    const double lookahead = computeLookahead();
    const bool overtake_active = updateOvertakeState();

    Waypoint target = line_->getLookaheadPoint(x_, y_, lookahead);
    double target_speed = target.speed;

    // Use local path from local_path_planner when overtake is active
    if (overtake_active && has_local_path_) {
      std::lock_guard<std::mutex> lock(local_path_mutex_);
      if (local_path_.size() >= 2) {
        size_t target_idx = getLookaheadIndexOnPath(x_, y_, lookahead, local_path_);
        Waypoint local_target = local_path_[target_idx];
        // Use position from local path, but get speed from waypoints
        target.x = local_target.x;
        target.y = local_target.y;

        if (has_local_speeds_ && local_path_speeds_.size() == local_path_.size() &&
            target_idx < local_path_speeds_.size()) {
          target_speed = local_path_speeds_[target_idx];
        }
      }
    }

    double steering = controller_->computeSteeringForTarget(
        x_, y_, yaw_, target.x, target.y, lookahead);

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

    std_msgs::msg::Bool flag_msg;
    flag_msg.data = overtake_active;
    overtake_pub_->publish(flag_msg);
  }

  double computeLookahead() const {
    double lookahead = pp_config_.lookahead_distance + pp_config_.lookahead_ratio * speed_;
    lookahead = std::clamp(lookahead, pp_config_.min_lookahead, pp_config_.max_lookahead);
    return lookahead;
  }

  bool updateOvertakeState() {
    if (!has_opp_pose_) {
      overtake_state_ = OvertakeState::NONE;
      return false;
    }
    const double dx = opp_x_ - x_;
    const double dy = opp_y_ - y_;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double heading_x = std::cos(yaw_);
    const double heading_y = std::sin(yaw_);
    const double dot = dx * heading_x + dy * heading_y;
    const bool is_ahead = dot > 0.0;

    if (overtake_state_ == OvertakeState::NONE) {
      if (is_ahead && dist <= overtake_trigger_distance_) {
        overtake_state_ = OvertakeState::PASS;
        overtake_start_time_ = now();
        RCLCPP_INFO(get_logger(), "Overtake triggered: distance=%.1f m", dist);
      }
    } else {
      if (!is_ahead && dist > overtake_complete_distance_) {
        overtake_state_ = OvertakeState::NONE;
        RCLCPP_INFO(get_logger(), "Overtake completed");
      }
    }

    return overtake_state_ != OvertakeState::NONE;
  }

  size_t getLookaheadIndexOnPath(double x, double y, double lookahead,
                                 const std::vector<Waypoint> &path) const {
    if (path.empty()) {
      return 0;
    }
    if (path.size() == 1) {
      return 0;
    }

    std::vector<double> cumulative;
    cumulative.reserve(path.size());
    cumulative.push_back(0.0);

    for (size_t i = 1; i < path.size(); ++i) {
      const double dx = path[i].x - path[i - 1].x;
      const double dy = path[i].y - path[i - 1].y;
      cumulative.push_back(cumulative.back() + std::sqrt(dx * dx + dy * dy));
    }

    int nearest_idx = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
      const double dx = path[i].x - x;
      const double dy = path[i].y - y;
      const double dist = dx * dx + dy * dy;
      if (dist < best_dist) {
        best_dist = dist;
        nearest_idx = static_cast<int>(i);
      }
    }

    const double target_dist = cumulative[static_cast<size_t>(nearest_idx)] + lookahead;
    auto it = std::lower_bound(cumulative.begin(), cumulative.end(), target_dist);
    size_t target_idx = static_cast<size_t>(std::distance(cumulative.begin(), it));
    if (target_idx >= path.size()) {
      target_idx = path.size() - 1;
    }
    return target_idx;
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> line_;
  std::shared_ptr<PurePursuitController> controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr opp_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr local_speed_sub_;
  rclcpp::Publisher<scenario_director::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double max_steering_ = 0.55;
  double overtake_trigger_distance_ = 20.0;
  double overtake_complete_distance_ = 8.0;
  PurePursuitConfig pp_config_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double speed_ = 0.0;
  bool has_pose_ = false;
  double opp_x_ = 0.0;
  double opp_y_ = 0.0;
  double opp_speed_ = 0.0;
  bool has_opp_pose_ = false;
  std::string last_frame_id_ = "map";

  std::mutex local_path_mutex_;
  std::vector<Waypoint> local_path_;
  std::vector<double> local_path_speeds_;
  bool has_local_path_ = false;
  bool has_local_speeds_ = false;

  enum class OvertakeState { NONE, PASS };
  OvertakeState overtake_state_ = OvertakeState::NONE;
  rclcpp::Time overtake_start_time_;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::ScenarioDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
