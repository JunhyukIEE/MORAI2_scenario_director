#include "scenario_director/track_boundary.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace scenario_director {

TrackBoundaryChecker::TrackBoundaryChecker(const std::string &map_dir, const std::string &map_yaml)
: map_dir_(map_dir) {
  config_ = loadMapConfig(map_yaml);
  map_image_ = loadMapImage();
  map_height_ = map_image_.rows;
  map_width_ = map_image_.cols;
}

MapConfig TrackBoundaryChecker::loadMapConfig(const std::string &map_yaml) const {
  const std::filesystem::path yaml_path = std::filesystem::path(map_dir_) / map_yaml;
  YAML::Node data = YAML::LoadFile(yaml_path.string());

  MapConfig cfg;
  const std::string image_name = data["image"].as<std::string>();
  cfg.image_path = (std::filesystem::path(map_dir_) / image_name).string();
  cfg.resolution = data["resolution"].as<double>();
  cfg.origin_x = data["origin"][0].as<double>();
  cfg.origin_y = data["origin"][1].as<double>();
  if (data["occupied_thresh"]) {
    cfg.occupied_thresh = data["occupied_thresh"].as<double>();
  }
  if (data["free_thresh"]) {
    cfg.free_thresh = data["free_thresh"].as<double>();
  }
  return cfg;
}

cv::Mat TrackBoundaryChecker::loadMapImage() const {
  cv::Mat img = cv::imread(config_.image_path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    throw std::runtime_error("Map image not found: " + config_.image_path);
  }
  return img;
}

const MapConfig &TrackBoundaryChecker::config() const {
  return config_;
}

std::pair<int, int> TrackBoundaryChecker::worldToPixel(double world_x, double world_y) const {
  const double rel_x = world_x - config_.origin_x;
  const double rel_y = world_y - config_.origin_y;

  int pixel_x = static_cast<int>(rel_x / config_.resolution);
  int pixel_y = static_cast<int>(rel_y / config_.resolution);

  pixel_y = map_height_ - 1 - pixel_y;
  return {pixel_x, pixel_y};
}

std::pair<double, double> TrackBoundaryChecker::pixelToWorld(int pixel_x, int pixel_y) const {
  int flipped_y = map_height_ - 1 - pixel_y;
  double world_x = pixel_x * config_.resolution + config_.origin_x;
  double world_y = flipped_y * config_.resolution + config_.origin_y;
  return {world_x, world_y};
}

bool TrackBoundaryChecker::isValidPixel(int pixel_x, int pixel_y) const {
  return pixel_x >= 0 && pixel_x < map_width_ && pixel_y >= 0 && pixel_y < map_height_;
}

bool TrackBoundaryChecker::isDrivable(double world_x, double world_y) const {
  auto [pixel_x, pixel_y] = worldToPixel(world_x, world_y);
  if (!isValidPixel(pixel_x, pixel_y)) {
    return false;
  }

  double pixel_value = map_image_.at<unsigned char>(pixel_y, pixel_x) / 255.0;

  // Keep parity with the Python logic: higher values treated as drivable.
  return pixel_value > config_.occupied_thresh;
}

bool TrackBoundaryChecker::isDrivableWithMargin(double world_x, double world_y, double margin) const {
  const std::vector<std::pair<double, double>> offsets = {
    {0.0, 0.0},
    {margin, 0.0}, {-margin, 0.0},
    {0.0, margin}, {0.0, -margin},
    {margin, margin}, {-margin, -margin},
    {margin, -margin}, {-margin, margin}
  };

  for (const auto &offset : offsets) {
    if (!isDrivable(world_x + offset.first, world_y + offset.second)) {
      return false;
    }
  }
  return true;
}

std::vector<bool> TrackBoundaryChecker::checkLineDrivable(const std::vector<double> &line_x,
                                                          const std::vector<double> &line_y,
                                                          double margin) const {
  const size_t n = line_x.size();
  std::vector<bool> drivable(n, true);
  for (size_t i = 0; i < n; ++i) {
    drivable[i] = isDrivableWithMargin(line_x[i], line_y[i], margin);
  }
  return drivable;
}

double TrackBoundaryChecker::getSafeOffset(double x, double y, double yaw,
                                           double desired_offset, double margin) const {
  const double normal_x = -std::sin(yaw);
  const double normal_y = std::cos(yaw);

  const double sign = desired_offset > 0.0 ? 1.0 : -1.0;
  const double max_offset = std::abs(desired_offset);

  const double step = 0.1;
  double safe_offset = 0.0;

  for (double offset = step; offset <= max_offset + 1e-6; offset += step) {
    const double check_x = x + sign * offset * normal_x;
    const double check_y = y + sign * offset * normal_y;

    if (isDrivableWithMargin(check_x, check_y, margin)) {
      safe_offset = offset;
    } else {
      break;
    }
  }

  return sign * safe_offset;
}

SafeLineGenerator::SafeLineGenerator(const TrackBoundaryChecker &checker)
: checker_(checker) {}

std::tuple<std::vector<double>, std::vector<double>, std::vector<bool>>
SafeLineGenerator::generateSafeOffsetLine(
    const std::vector<double> &base_x,
    const std::vector<double> &base_y,
    const std::vector<double> &base_yaw,
    const std::vector<double> &base_speed,
    double desired_offset,
    double vehicle_width) const {
  (void)base_speed;
  const size_t n = base_x.size();
  std::vector<double> offset_x(n);
  std::vector<double> offset_y(n);

  for (size_t i = 0; i < n; ++i) {
    const double normal_x = -std::sin(base_yaw[i]);
    const double normal_y = std::cos(base_yaw[i]);
    offset_x[i] = base_x[i] + desired_offset * normal_x;
    offset_y[i] = base_y[i] + desired_offset * normal_y;
  }

  std::vector<bool> valid_mask = checker_.checkLineDrivable(offset_x, offset_y, vehicle_width);

  for (size_t i = 0; i < n; ++i) {
    if (!valid_mask[i]) {
      const double safe_offset = checker_.getSafeOffset(
          base_x[i], base_y[i], base_yaw[i], desired_offset, vehicle_width);
      const double normal_x = -std::sin(base_yaw[i]);
      const double normal_y = std::cos(base_yaw[i]);
      offset_x[i] = base_x[i] + safe_offset * normal_x;
      offset_y[i] = base_y[i] + safe_offset * normal_y;
    }
  }

  return {offset_x, offset_y, valid_mask};
}

}  // namespace scenario_director
