#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/line_manager.hpp"
#include "scenario_director/pure_pursuit.hpp"

#include <cmath>
#include <filesystem>
#include <functional>

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
    declare_parameter<std::string>("line.optimal_path", "lines/optimal_line.csv");
    declare_parameter<double>("npc.speed_ratio", 0.5);
    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.max_steering", 0.55);
    declare_parameter<double>("pure_pursuit.lookahead_distance", 6.0);
    declare_parameter<double>("pure_pursuit.min_lookahead", 3.0);
    declare_parameter<double>("pure_pursuit.max_lookahead", 10.0);
    declare_parameter<double>("pure_pursuit.lookahead_ratio", 0.2);

    const std::string line_path = resolvePath(get_parameter("line.optimal_path").as_string(),
                                              "scenario_director");
    speed_ratio_ = get_parameter("npc.speed_ratio").as_double();

    PurePursuitConfig cfg;
    cfg.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    cfg.max_steering = get_parameter("vehicle.max_steering").as_double();
    cfg.lookahead_distance = get_parameter("pure_pursuit.lookahead_distance").as_double();
    cfg.min_lookahead = get_parameter("pure_pursuit.min_lookahead").as_double();
    cfg.max_lookahead = get_parameter("pure_pursuit.max_lookahead").as_double();
    cfg.lookahead_ratio = get_parameter("pure_pursuit.lookahead_ratio").as_double();

    line_manager_ = std::make_shared<LineManager>();
    try {
      line_manager_->loadOptimalLine(line_path);
      line_ = line_manager_->getLine("optimal");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load line: %s", e.what());
    }

    controller_ = std::make_shared<PurePursuitController>(cfg);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/npc/odom", 10, std::bind(&NPCControllerNode::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/npc/ctrl_cmd", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&NPCControllerNode::controlLoop, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!has_pose_) {
      has_pose_ = true;
    }
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  }

  void controlLoop() {
    if (!has_pose_ || !line_) {
      return;
    }

    auto [steering, target_speed] = controller_->compute(x_, y_, yaw_, speed_, *line_);
    target_speed *= speed_ratio_;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed;
    cmd.angular.z = steering;
    cmd_pub_->publish(cmd);
  }

  std::shared_ptr<LineManager> line_manager_;
  std::shared_ptr<RacingLine> line_;
  std::shared_ptr<PurePursuitController> controller_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double speed_ratio_ = 0.5;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  double speed_ = 0.0;
  bool has_pose_ = false;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::NPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
