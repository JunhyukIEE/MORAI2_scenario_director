#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <morai_msgs/msg/float64_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

#include "dataset_logger/data_logger.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <random>
#include <limits>
#include <mutex>

namespace dataset_logger {

class DatasetLoggerNode : public rclcpp::Node {
public:
  DatasetLoggerNode()
  : rclcpp::Node("dataset_logger") {
    declare_parameter<std::string>("logging.output_dir", "output/dataset");
    declare_parameter<std::string>("logging.image_format", "jpg");
    declare_parameter<int>("logging.jpg_quality", 90);

    declare_parameter<bool>("split.enable", true);
    declare_parameter<std::string>("split.mode", "episode");
    declare_parameter<double>("split.train_ratio", 0.8);
    declare_parameter<double>("split.val_ratio", 0.1);
    declare_parameter<double>("split.test_ratio", 0.1);
    declare_parameter<int>("split.seed", 42);


    output_dir_ = get_parameter("logging.output_dir").as_string();
    image_format_ = get_parameter("logging.image_format").as_string();
    jpg_quality_ = get_parameter("logging.jpg_quality").as_int();

    split_enabled_ = get_parameter("split.enable").as_bool();
    split_mode_ = get_parameter("split.mode").as_string();
    train_ratio_ = get_parameter("split.train_ratio").as_double();
    val_ratio_ = get_parameter("split.val_ratio").as_double();
    test_ratio_ = get_parameter("split.test_ratio").as_double();
    split_seed_ = get_parameter("split.seed").as_int();


    initLoggers();

    ego_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/ego/odom", 10, std::bind(&DatasetLoggerNode::egoOdomCallback, this, std::placeholders::_1));
    ego_vel_sub_ = create_subscription<morai_msgs::msg::Float64Stamped>(
      "/ego/vehicle/velocity", 10, std::bind(&DatasetLoggerNode::egoVelCallback, this, std::placeholders::_1));
    ego_steer_sub_ = create_subscription<morai_msgs::msg::Float64Stamped>(
      "/ego/vehicle/steering_state", 10, std::bind(&DatasetLoggerNode::egoSteerCallback, this, std::placeholders::_1));

    opp_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/opponent/odom", 10, std::bind(&DatasetLoggerNode::oppOdomCallback, this, std::placeholders::_1));
    opp_vel_sub_ = create_subscription<morai_msgs::msg::Float64Stamped>(
      "/opponent/vehicle/velocity", 10, std::bind(&DatasetLoggerNode::oppVelCallback, this, std::placeholders::_1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/ego/camera/image_raw", 10, std::bind(&DatasetLoggerNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Dataset logger started: %s", output_dir_.c_str());
  }

  ~DatasetLoggerNode() override {
    closeLoggers();
  }

private:
  void initLoggers() {
    std::filesystem::path base(output_dir_);
    if (!base.is_absolute()) {
      base = std::filesystem::current_path() / base;
    }

    const double total = std::max(0.0, train_ratio_) + std::max(0.0, val_ratio_) + std::max(0.0, test_ratio_);
    if (total <= 0.0) {
      train_ratio_ = 0.8;
      val_ratio_ = 0.1;
      test_ratio_ = 0.1;
    } else {
      train_ratio_ /= total;
      val_ratio_ /= total;
      test_ratio_ /= total;
    }

    rng_.seed(static_cast<unsigned int>(split_seed_));
    dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    if (!split_enabled_) {
      logger_ = std::make_shared<DataLogger>(base.string(), image_format_, jpg_quality_);
      episode_split_ = "all";
      return;
    }

    train_logger_ = std::make_shared<DataLogger>((base / "train").string(), image_format_, jpg_quality_);
    val_logger_ = std::make_shared<DataLogger>((base / "val").string(), image_format_, jpg_quality_);
    test_logger_ = std::make_shared<DataLogger>((base / "test").string(), image_format_, jpg_quality_);

    if (split_mode_ == "episode") {
      const double r = dist_(rng_);
      if (r < train_ratio_) {
        episode_split_ = "train";
      } else if (r < train_ratio_ + val_ratio_) {
        episode_split_ = "val";
      } else {
        episode_split_ = "test";
      }
    } else {
      episode_split_ = "mixed";
    }
  }

  void closeLoggers() {
    if (logger_) {
      logger_->close();
    }
    if (train_logger_) {
      train_logger_->close();
    }
    if (val_logger_) {
      val_logger_->close();
    }
    if (test_logger_) {
      test_logger_->close();
    }
  }

  std::shared_ptr<DataLogger> selectLogger() {
    if (!split_enabled_) {
      return logger_;
    }

    if (split_mode_ == "episode") {
      if (episode_split_ == "train") {
        return train_logger_;
      }
      if (episode_split_ == "val") {
        return val_logger_;
      }
      return test_logger_;
    }

    double r = 0.0;
    if (split_mode_ == "sequential") {
      r = (split_counter_ % 100) / 100.0;
    } else {
      r = dist_(rng_);
    }
    ++split_counter_;

    if (r < train_ratio_) {
      return train_logger_;
    }
    if (r < train_ratio_ + val_ratio_) {
      return val_logger_;
    }
    return test_logger_;
  }


  static double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    ego_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
    has_ego_ = true;
  }

  void egoVelCallback(const morai_msgs::msg::Float64Stamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ego_speed_ = msg->data;
  }

  void egoSteerCallback(const morai_msgs::msg::Float64Stamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    ego_steering_ = msg->data;
  }

  void oppOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    opp_x_ = msg->pose.pose.position.x;
    opp_y_ = msg->pose.pose.position.y;
    has_opp_ = true;
  }

  void oppVelCallback(const morai_msgs::msg::Float64Stamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    opp_speed_ = msg->data;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat image;
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      image = cv_ptr->image;
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Image conversion failed: %s", e.what());
      return;
    }

    auto logger = selectLogger();
    if (!logger) {
      return;
    }

    DataSample sample;
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_ego_) {
        return;
      }

      sample.frame_id = logger->frameId();
      sample.timestamp = timestamp;
      sample.ego_x = ego_x_;
      sample.ego_y = ego_y_;
      sample.ego_yaw = ego_yaw_;
      sample.ego_speed = ego_speed_;
      sample.steering_angle = ego_steering_;

      if (has_opp_) {
        sample.opp_x = opp_x_;
        sample.opp_y = opp_y_;
        sample.opp_speed = opp_speed_;
        const double dx = ego_x_ - opp_x_;
        const double dy = ego_y_ - opp_y_;
        sample.opp_distance = std::sqrt(dx * dx + dy * dy);
      }

    }

    logger->log(timestamp, &image, sample);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<morai_msgs::msg::Float64Stamped>::SharedPtr ego_vel_sub_;
  rclcpp::Subscription<morai_msgs::msg::Float64Stamped>::SharedPtr ego_steer_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom_sub_;
  rclcpp::Subscription<morai_msgs::msg::Float64Stamped>::SharedPtr opp_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::mutex mutex_;
  bool has_ego_ = false;
  bool has_opp_ = false;
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;
  double ego_yaw_ = 0.0;
  double ego_speed_ = 0.0;
  double ego_steering_ = std::numeric_limits<double>::quiet_NaN();
  double opp_x_ = -1.0;
  double opp_y_ = -1.0;
  double opp_speed_ = -1.0;

  std::string output_dir_;
  std::string image_format_;
  int jpg_quality_ = 90;

  bool split_enabled_ = true;
  std::string split_mode_ = "episode";
  double train_ratio_ = 0.8;
  double val_ratio_ = 0.1;
  double test_ratio_ = 0.1;
  int split_seed_ = 42;
  size_t split_counter_ = 0;

  std::string episode_split_ = "train";

  std::shared_ptr<DataLogger> logger_;
  std::shared_ptr<DataLogger> train_logger_;
  std::shared_ptr<DataLogger> val_logger_;
  std::shared_ptr<DataLogger> test_logger_;

  std::mt19937 rng_;
  std::uniform_real_distribution<double> dist_;
};

}  // namespace dataset_logger

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dataset_logger::DatasetLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
