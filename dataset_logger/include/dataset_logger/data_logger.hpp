#pragma once

#include <opencv2/core.hpp>

#include <fstream>
#include <memory>
#include <string>

namespace dataset_logger {

struct DataSample {
  int frame_id = 0;
  double timestamp = 0.0;

  double ego_x = 0.0;
  double ego_y = 0.0;
  double ego_yaw = 0.0;
  double ego_speed = 0.0;
  double steering_angle = 0.0;

  double opp_x = -1.0;
  double opp_y = -1.0;
  double opp_speed = -1.0;
  double opp_distance = -1.0;

};

class DataLogger {
public:
  DataLogger(const std::string &output_dir,
             const std::string &image_format = "jpg",
             int jpg_quality = 90);

  void log(double timestamp, const cv::Mat *image, const DataSample &sample);
  void close();

  int frameId() const;

  static std::string nowIsoString();

private:
  void initCsv();
  void saveMetadata();
  std::string output_dir_;
  std::string image_format_;
  int jpg_quality_ = 90;

  std::string image_dir_;
  std::string csv_path_;

  std::ofstream csv_file_;

  int frame_id_ = 0;
  int total_frames_ = 0;
  std::string created_at_;
};

class SessionManager {
public:
  explicit SessionManager(const std::string &base_dir);

  std::shared_ptr<DataLogger> startEpisode(const std::string &image_format, int jpg_quality);
  void endEpisode();

  std::string sessionId() const;
  std::string sessionDir() const;
  int totalEpisodes() const;

private:
  std::string base_dir_;
  std::string session_id_;
  std::string session_dir_;
  int episode_count_ = 0;
  std::shared_ptr<DataLogger> current_logger_;
};

}  // namespace dataset_logger
