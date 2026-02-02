#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "scenario_director/overtake_planner.hpp"

#include <filesystem>
#include <string>

namespace scenario_director {

class OvertakePlannerNode : public rclcpp::Node {
public:
  OvertakePlannerNode() : rclcpp::Node("overtake_planner") {
    declare_parameter<std::string>("map.map_dir", "map");
    declare_parameter<std::string>("map.map_yaml", "Sangam_map.yaml");
    declare_parameter<std::string>("map.waypoints", "waypoints.csv");
    declare_parameter<std::string>("output.dir", "out");

    declare_parameter<double>("safety_margin_m", 0.25);
    declare_parameter<double>("alat_max", 6.0);
    declare_parameter<double>("amax", 2.0);
    declare_parameter<double>("bmax", 3.0);
    declare_parameter<int>("num_candidates_per_corner", 7);
    declare_parameter<double>("curvature_threshold_percentile", 80.0);

    const std::string share = ament_index_cpp::get_package_share_directory("scenario_director");
    const std::string map_dir_rel = get_parameter("map.map_dir").as_string();
    const std::string map_yaml = get_parameter("map.map_yaml").as_string();
    const std::string waypoints_rel = get_parameter("map.waypoints").as_string();
    const std::string out_rel = get_parameter("output.dir").as_string();

    std::filesystem::path map_dir_path = map_dir_rel;
    if (!map_dir_path.is_absolute()) {
      map_dir_path = std::filesystem::path(share) / map_dir_rel;
    }
    std::filesystem::path waypoints_path = waypoints_rel;
    if (!waypoints_path.is_absolute()) {
      waypoints_path = map_dir_path / waypoints_rel;
    }
    std::filesystem::path out_dir_path = out_rel;
    if (!out_dir_path.is_absolute()) {
      out_dir_path = std::filesystem::path(share) / out_rel;
    }

    const std::string map_dir = map_dir_path.string();
    const std::string waypoints = waypoints_path.string();
    const std::string out_dir = out_dir_path.string();

    OvertakeConfig cfg;
    cfg.safety_margin_m = get_parameter("safety_margin_m").as_double();
    cfg.alat_max = get_parameter("alat_max").as_double();
    cfg.amax = get_parameter("amax").as_double();
    cfg.bmax = get_parameter("bmax").as_double();
    cfg.num_candidates_per_corner = get_parameter("num_candidates_per_corner").as_int();
    cfg.curvature_threshold_percentile = get_parameter("curvature_threshold_percentile").as_double();

    planner_ = std::make_shared<OvertakePlanner>(map_dir, map_yaml, waypoints, cfg);
    planner_->generate(out_dir);

    RCLCPP_INFO(get_logger(), "Overtake planner outputs written to %s", out_dir.c_str());
  }

private:
  std::shared_ptr<OvertakePlanner> planner_;
};

}  // namespace scenario_director

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scenario_director::OvertakePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
