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

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
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
    declare_parameter<double>("overtake.trigger_distance", 20.0);
    declare_parameter<double>("overtake.complete_distance", 8.0);
    declare_parameter<double>("overtake.side_offset", 1.5);
    declare_parameter<double>("overtake.dummy_offset", 0.8);
    declare_parameter<double>("overtake.dummy_duration", 0.6);
    declare_parameter<double>("overtake.entry_speed_scale", 0.85);
    declare_parameter<double>("overtake.exit_speed_scale", 1.10);
    declare_parameter<double>("overtake.late_brake_distance", 6.0);
    declare_parameter<double>("overtake.late_brake_fast_scale", 1.05);
    declare_parameter<double>("overtake.late_brake_slow_scale", 0.85);
    declare_parameter<bool>("overtake.outside_enabled", true);
    declare_parameter<int>("overtake.local_path_points", 60);
    declare_parameter<int>("overtake.local_path_stride", 1);
    declare_parameter<std::string>("overtake.local_path_topic", "/local_path");

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
    overtake_trigger_distance_ = get_parameter("overtake.trigger_distance").as_double();
    overtake_complete_distance_ = get_parameter("overtake.complete_distance").as_double();
    overtake_side_offset_ = get_parameter("overtake.side_offset").as_double();
    overtake_dummy_offset_ = get_parameter("overtake.dummy_offset").as_double();
    overtake_dummy_duration_ = get_parameter("overtake.dummy_duration").as_double();
    overtake_entry_speed_scale_ = get_parameter("overtake.entry_speed_scale").as_double();
    overtake_exit_speed_scale_ = get_parameter("overtake.exit_speed_scale").as_double();
    overtake_late_brake_distance_ = get_parameter("overtake.late_brake_distance").as_double();
    overtake_late_brake_fast_scale_ = get_parameter("overtake.late_brake_fast_scale").as_double();
    overtake_late_brake_slow_scale_ = get_parameter("overtake.late_brake_slow_scale").as_double();
    overtake_outside_enabled_ = get_parameter("overtake.outside_enabled").as_bool();
    pp_config_ = cfg;

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10, std::bind(&ScenarioDirectorNode::odomCallback, this, std::placeholders::_1));
    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10, std::bind(&ScenarioDirectorNode::velocityCallback, this, std::placeholders::_1));

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opponent/odom", 10, std::bind(&ScenarioDirectorNode::oppOdomCallback, this, std::placeholders::_1));
    opp_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/opponent/vehicle/velocity", 10, std::bind(&ScenarioDirectorNode::oppVelocityCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<scenario_director::msg::VehicleCmd>("/ego/ctrl_cmd", 10);
    overtake_pub_ = create_publisher<std_msgs::msg::Bool>("/ego/overtake_flag", 10);
    local_path_points_ = std::max(2, static_cast<int>(
        get_parameter("overtake.local_path_points").as_int()));
    local_path_stride_ = std::max(1, static_cast<int>(
        get_parameter("overtake.local_path_stride").as_int()));
    local_path_topic_ = get_parameter("overtake.local_path_topic").as_string();
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>(local_path_topic_, 1);

    const double loop_hz = get_parameter("control.loop_hz").as_double();
    const int period_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, loop_hz)));
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                               std::bind(&ScenarioDirectorNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Scenario Director (minimal waypoint mode) initialized");
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

  void controlLoop() {
    if (!has_pose_ || !line_) {
      return;
    }

    const double lookahead = computeLookahead();
    const bool overtake_active = updateOvertakeState();
    const auto local_path = buildLocalPath(overtake_active);
    publishLocalPath(local_path);

    Waypoint target = line_->getLookaheadPoint(x_, y_, lookahead);
    if (overtake_active && local_path.size() >= 2) {
      target = getLookaheadOnPath(x_, y_, lookahead, local_path);
    } else if (overtake_active) {
      applyOvertakeOffset(target);
    }

    double steering = controller_->computeSteeringForTarget(
        x_, y_, yaw_, target.x, target.y, lookahead);
    steering *= pp_config_.steering_scale;

    double target_speed = target.speed;
    if (overtake_active) {
      target_speed = adjustOvertakeSpeed(target_speed);
    }

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
        const double lateral = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
        pass_side_sign_ = lateral > 0.0 ? -1.0 : 1.0;
        if (overtake_outside_enabled_) {
          const int turn_sign = estimateTurnSign();
          if (turn_sign != 0) {
            pass_side_sign_ = (turn_sign > 0) ? -1.0 : 1.0;
          }
        }
        dummy_side_sign_ = -pass_side_sign_;
        overtake_state_ = overtake_dummy_duration_ > 0.0 ? OvertakeState::DUMMY : OvertakeState::PASS;
        overtake_start_time_ = now();
      }
    } else {
      if (!is_ahead && dist > overtake_complete_distance_) {
        overtake_state_ = OvertakeState::NONE;
      }
      if (overtake_state_ == OvertakeState::DUMMY) {
        const double elapsed = (now() - overtake_start_time_).seconds();
        if (elapsed >= overtake_dummy_duration_) {
          overtake_state_ = OvertakeState::PASS;
        }
      }
    }

    return overtake_state_ != OvertakeState::NONE;
  }

  int estimateTurnSign() const {
    if (!line_) {
      return 0;
    }
    const int idx0 = line_->getNearestIndex(x_, y_);
    const int idx1 = idx0 + 3;
    const int idx2 = idx0 + 6;
    if (line_->size() < 3) {
      return 0;
    }
    const Waypoint p0 = line_->getWaypoint(idx0);
    const Waypoint p1 = line_->getWaypoint(idx1);
    const Waypoint p2 = line_->getWaypoint(idx2);
    const double v1x = p1.x - p0.x;
    const double v1y = p1.y - p0.y;
    const double v2x = p2.x - p1.x;
    const double v2y = p2.y - p1.y;
    const double cross = v1x * v2y - v1y * v2x;
    if (std::abs(cross) < 1e-3) {
      return 0;
    }
    return cross > 0.0 ? 1 : -1;
  }

  void applyOvertakeOffset(Waypoint &target) const {
    const double dx = target.x - x_;
    const double dy = target.y - y_;
    const double target_heading = std::atan2(dy, dx);
    const double left_x = -std::sin(target_heading);
    const double left_y = std::cos(target_heading);

    double side_sign = pass_side_sign_;
    double offset = overtake_side_offset_;
    if (overtake_state_ == OvertakeState::DUMMY) {
      side_sign = dummy_side_sign_;
      offset = overtake_dummy_offset_;
    }
    target.x += left_x * offset * side_sign;
    target.y += left_y * offset * side_sign;
  }

  void applyLateralOffset(Waypoint &wp, double heading, double side_sign, double offset) const {
    const double left_x = -std::sin(heading);
    const double left_y = std::cos(heading);
    wp.x += left_x * offset * side_sign;
    wp.y += left_y * offset * side_sign;
  }

  std::vector<Waypoint> buildLocalPath(bool overtake_active) const {
    std::vector<Waypoint> result;
    if (!line_) {
      return result;
    }

    const int base_idx = line_->getNearestIndex(x_, y_);
    const int stride = std::max(1, local_path_stride_);
    const int count = std::max(2, local_path_points_);

    double side_sign = pass_side_sign_;
    double offset = overtake_side_offset_;
    if (overtake_state_ == OvertakeState::DUMMY) {
      side_sign = dummy_side_sign_;
      offset = overtake_dummy_offset_;
    }
    if (!overtake_active) {
      offset = 0.0;
    }

    for (int i = 0; i < count; ++i) {
      const int idx = base_idx + i * stride;
      Waypoint wp = line_->getWaypoint(idx);
      const Waypoint next = line_->getWaypoint(idx + 1);
      double heading = std::atan2(next.y - wp.y, next.x - wp.x);
      if (std::abs(next.x - wp.x) < 1e-6 && std::abs(next.y - wp.y) < 1e-6) {
        heading = wp.yaw;
      }

      if (offset != 0.0) {
        applyLateralOffset(wp, heading, side_sign, offset);
      }

      wp.yaw = heading;
      result.push_back(wp);
    }

    return result;
  }

  void publishLocalPath(const std::vector<Waypoint> &local_path) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = last_frame_id_.empty() ? "map" : last_frame_id_;

    for (const auto &wp : local_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = wp.x;
      pose.pose.position.y = wp.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(wp.yaw);
      path.poses.push_back(pose);
    }

    local_path_pub_->publish(path);
  }

  Waypoint getLookaheadOnPath(double x, double y, double lookahead,
                              const std::vector<Waypoint> &path) const {
    if (path.empty()) {
      return {};
    }
    if (path.size() == 1) {
      return path.front();
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
    return path[target_idx];
  }

  double adjustOvertakeSpeed(double target_speed) const {
    const double dx = opp_x_ - x_;
    const double dy = opp_y_ - y_;
    const double dist = std::sqrt(dx * dx + dy * dy);

    target_speed *= overtake_entry_speed_scale_;
    if (dist > overtake_late_brake_distance_) {
      target_speed *= overtake_late_brake_fast_scale_;
    } else {
      target_speed *= overtake_late_brake_slow_scale_;
    }
    if (overtake_state_ == OvertakeState::PASS && dist > overtake_complete_distance_) {
      target_speed *= overtake_exit_speed_scale_;
    }
    return target_speed;
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> line_;
  std::shared_ptr<PurePursuitController> controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr opp_vel_sub_;
  rclcpp::Publisher<scenario_director::msg::VehicleCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtake_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double max_steering_ = 0.55;
  double overtake_trigger_distance_ = 20.0;
  double overtake_complete_distance_ = 8.0;
  double overtake_side_offset_ = 1.5;
  double overtake_dummy_offset_ = 0.8;
  double overtake_dummy_duration_ = 0.6;
  double overtake_entry_speed_scale_ = 0.85;
  double overtake_exit_speed_scale_ = 1.10;
  double overtake_late_brake_distance_ = 6.0;
  double overtake_late_brake_fast_scale_ = 1.05;
  double overtake_late_brake_slow_scale_ = 0.85;
  bool overtake_outside_enabled_ = true;
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
  int local_path_points_ = 60;
  int local_path_stride_ = 1;
  std::string local_path_topic_ = "/local_path";

  enum class OvertakeState { NONE, DUMMY, PASS };
  OvertakeState overtake_state_ = OvertakeState::NONE;
  rclcpp::Time overtake_start_time_;
  double pass_side_sign_ = 1.0;
  double dummy_side_sign_ = -1.0;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::ScenarioDirectorNode>());
  rclcpp::shutdown();
  return 0;
}
