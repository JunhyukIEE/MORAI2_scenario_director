#include "dataset_logger/data_logger.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

namespace dataset_logger {

DataLogger::DataLogger(const std::string &output_dir,
                       const std::string &image_format,
                       int jpg_quality)
: output_dir_(output_dir),
  image_format_(image_format),
  jpg_quality_(jpg_quality) {
  std::filesystem::create_directories(output_dir_);
  image_dir_ = (std::filesystem::path(output_dir_) / "images").string();
  std::filesystem::create_directories(image_dir_);

  csv_path_ = (std::filesystem::path(output_dir_) / "labels.csv").string();
  created_at_ = nowIsoString();
  initCsv();
}

void DataLogger::initCsv() {
  csv_file_.open(csv_path_);
  csv_file_ << "frame_id,timestamp,image_path,"
            << "ego_x,ego_y,ego_yaw,ego_speed,steering_angle,"
            << "opp_x,opp_y,opp_speed,opp_distance\n";
}

void DataLogger::log(double timestamp, const cv::Mat *image, const DataSample &sample) {
  std::ostringstream name_builder;
  name_builder << std::setw(8) << std::setfill('0') << frame_id_;
  const std::string image_filename = name_builder.str();
  const std::string image_path = image_dir_ + "/" + image_filename + "." + image_format_;

  if (image && !image->empty()) {
    std::vector<int> params;
    if (image_format_ == "jpg" || image_format_ == "jpeg") {
      params = {cv::IMWRITE_JPEG_QUALITY, jpg_quality_};
    }
    cv::imwrite(image_path, *image, params);
  }

  csv_file_ << frame_id_ << ","
            << timestamp << ","
            << "images/" << image_filename << "." << image_format_ << ","
            << sample.ego_x << "," << sample.ego_y << "," << sample.ego_yaw << ","
            << sample.ego_speed << "," << sample.steering_angle << ","
            << sample.opp_x << "," << sample.opp_y << "," << sample.opp_speed << ","
            << sample.opp_distance << "\n";

  total_frames_ = frame_id_ + 1;
  ++frame_id_;

  if (frame_id_ % 100 == 0) {
    csv_file_.flush();
  }
}

void DataLogger::saveMetadata() {
  const std::string meta_path = (std::filesystem::path(output_dir_) / "metadata.json").string();
  std::ofstream meta(meta_path);
  meta << "{\n";
  meta << "  \"created_at\": \"" << created_at_ << "\",\n";
  meta << "  \"image_format\": \"" << image_format_ << "\",\n";
  meta << "  \"total_frames\": " << total_frames_ << "\n";
  meta << "}\n";
}

void DataLogger::close() {
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
  }
  saveMetadata();
}

int DataLogger::frameId() const {
  return frame_id_;
}

std::string DataLogger::nowIsoString() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &time);
#else
  localtime_r(&time, &tm);
#endif
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
  return ss.str();
}

SessionManager::SessionManager(const std::string &base_dir)
: base_dir_(base_dir) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &time);
#else
  localtime_r(&time, &tm);
#endif
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  session_id_ = ss.str();
  session_dir_ = (std::filesystem::path(base_dir_) / session_id_).string();
  std::filesystem::create_directories(session_dir_);
}

std::shared_ptr<DataLogger> SessionManager::startEpisode(const std::string &image_format,
                                                         int jpg_quality) {
  std::ostringstream episode_name;
  episode_name << "episode_" << std::setw(4) << std::setfill('0') << episode_count_;
  std::string episode_dir = (std::filesystem::path(session_dir_) / episode_name.str()).string();

  current_logger_ = std::make_shared<DataLogger>(episode_dir, image_format, jpg_quality);
  ++episode_count_;
  return current_logger_;
}

void SessionManager::endEpisode() {
  if (current_logger_) {
    current_logger_->close();
    current_logger_.reset();
  }
}

std::string SessionManager::sessionId() const {
  return session_id_;
}

std::string SessionManager::sessionDir() const {
  return session_dir_;
}

int SessionManager::totalEpisodes() const {
  return episode_count_;
}

}  // namespace dataset_logger
