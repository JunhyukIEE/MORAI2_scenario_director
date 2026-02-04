#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/local_path_planner.hpp"

#include <filesystem>
#include <mutex>

namespace scenario_director {

class LocalPathPlannerNode : public rclcpp::Node {
public:
  LocalPathPlannerNode() : rclcpp::Node("local_path_planner") {
    // Declare parameters
    declare_parameter<std::string>("map.map_dir", "map");
    declare_parameter<std::string>("map.map_yaml", "Sangam_map.yaml");
    declare_parameter<std::string>("map.waypoints", "waypoints.csv");

    // Vehicle parameters
    declare_parameter<double>("vehicle.wheelbase", 2.7);
    declare_parameter<double>("vehicle.max_steer", 0.489);
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

    config.dt = get_parameter("dt").as_double();
    config.horizon_s = get_parameter("horizon_s").as_double();
    config.path_ahead_m = get_parameter("path_ahead_m").as_double();

    config.lateral_offsets = get_parameter("lateral_offsets").as_double_array();
    config.speed_scales = get_parameter("speed_scales").as_double_array();

    config.offroad_check_stride = get_parameter("offroad_check_stride").as_int();
    config.overtake_margin_s = get_parameter("overtake_margin_s").as_double();
    config.near_pass_bonus_dist = get_parameter("near_pass_bonus_dist").as_double();
    config.lat_change_penalty = get_parameter("lat_change_penalty").as_double();

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

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/opponent/odom", 10,
        std::bind(&LocalPathPlannerNode::oppOdomCallback, this, std::placeholders::_1));

    // Publishers
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planner/path", 10);
    candidate_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_planner/candidate_path", 10);
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

  void oppOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    opp_x_ = msg->pose.pose.position.x;
    opp_y_ = msg->pose.pose.position.y;
    opp_v_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    opp_received_ = true;
  }

  void planningCallback() {
    double ego_x, ego_y, ego_yaw, ego_v;
    double opp_x, opp_y, opp_v;
    bool can_plan = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (ego_received_ && opp_received_) {
        ego_x = ego_x_;
        ego_y = ego_y_;
        ego_yaw = ego_yaw_;
        ego_v = ego_v_;
        opp_x = opp_x_;
        opp_y = opp_y_;
        opp_v = opp_v_;
        can_plan = true;
      }
    }

    if (!can_plan) {
      return;
    }

    // Run planner
    auto result = planner_->plan(ego_x, ego_y, ego_yaw, ego_v, opp_x, opp_y, opp_v);

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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr reward_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lat_offset_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Data mutex and state
  std::mutex data_mutex_;
  double ego_x_ = 0.0, ego_y_ = 0.0, ego_yaw_ = 0.0, ego_v_ = 0.0;
  double opp_x_ = 0.0, opp_y_ = 0.0, opp_v_ = 0.0;
  bool ego_received_ = false;
  bool opp_received_ = false;
};

}  // namespace scenario_director

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::LocalPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
