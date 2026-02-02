#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <tuple>
#include <vector>

namespace scenario_director {

struct MapConfig {
  std::string image_path;
  double resolution = 1.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
};

class TrackBoundaryChecker {
public:
  TrackBoundaryChecker(const std::string &map_dir, const std::string &map_yaml = "Sangam_map.yaml");

  std::pair<int, int> worldToPixel(double world_x, double world_y) const;
  std::pair<double, double> pixelToWorld(int pixel_x, int pixel_y) const;

  bool isValidPixel(int pixel_x, int pixel_y) const;
  bool isDrivable(double world_x, double world_y) const;
  bool isDrivableWithMargin(double world_x, double world_y, double margin = 0.5) const;

  std::vector<bool> checkLineDrivable(const std::vector<double> &line_x,
                                      const std::vector<double> &line_y,
                                      double margin = 0.3) const;

  double getSafeOffset(double x, double y, double yaw,
                       double desired_offset, double margin = 0.3) const;

  const MapConfig &config() const;
  const cv::Mat &mapImage() const;
  int mapWidth() const;
  int mapHeight() const;

private:
  MapConfig loadMapConfig(const std::string &map_yaml) const;
  cv::Mat loadMapImage() const;

  std::string map_dir_;
  MapConfig config_;
  cv::Mat map_image_;
  int map_width_ = 0;
  int map_height_ = 0;
};

class SafeLineGenerator {
public:
  explicit SafeLineGenerator(const TrackBoundaryChecker &checker);

  std::tuple<std::vector<double>, std::vector<double>, std::vector<bool>> generateSafeOffsetLine(
      const std::vector<double> &base_x,
      const std::vector<double> &base_y,
      const std::vector<double> &base_yaw,
      const std::vector<double> &base_speed,
      double desired_offset,
      double vehicle_width = 0.5) const;

private:
  const TrackBoundaryChecker &checker_;
};

}  // namespace scenario_director
