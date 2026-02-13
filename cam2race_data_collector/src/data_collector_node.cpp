#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "morai_msgs/msg/float64_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// Namespaces
using namespace std::chrono_literals;
namespace fs = std::filesystem;

class DataCollectorNode : public rclcpp::Node
{
public:
  DataCollectorNode()
  : Node("data_collector_node")
  {
    // Parameters
    this->declare_parameter<std::string>("save_path", "/home/sws/Documents/data");
    this->get_parameter("save_path", save_path_);
    
    RCLCPP_INFO(this->get_logger(), "Save path: %s", save_path_.c_str());

    // Initialize directories
    init_directories();

    // Initialize CSV file
    init_csv();

    // QoS Setting (Reliable for Camera data to match Simulator)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 10;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    // Subscribers with Message Filters
    // Topics:
    // cam0: front, cam1: front_left, cam4: front_right, cam2: rear_left, cam3: rear_right
    // Note: Variable names follow logical index, mapping handles the topic name.

    cam0_sub_.subscribe(this, "/sensing/camera/front/image_raw", qos_profile);
    cam1_sub_.subscribe(this, "/sensing/camera/front_left/image_raw", qos_profile);
    cam4_sub_.subscribe(this, "/sensing/camera/front_right/image_raw", qos_profile);
    cam2_sub_.subscribe(this, "/sensing/camera/rear_left/image_raw", qos_profile);
    cam3_sub_.subscribe(this, "/sensing/camera/rear_right/image_raw", qos_profile);

    velocity_sub_.subscribe(this, "/Ego/vehicle/status/velocity_status", qos_profile);
    steering_sub_.subscribe(this, "/Ego/vehicle/status/steering_status", qos_profile);

    // Synchronizer
    // Sync 7 topics: 5 Cameras + VelocityStamped + SteeringStamped
    // Queue size 20, roughly 0.1s~0.2s sync window depending on freq
    sync_ = std::make_shared<Sync>(MySyncPolicy(20), 
      cam0_sub_, cam1_sub_, cam4_sub_, cam2_sub_, cam3_sub_, velocity_sub_, steering_sub_);
      
    sync_->registerCallback(std::bind(&DataCollectorNode::callback, this, 
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));

    RCLCPP_INFO(this->get_logger(), "Data Collector Node Initialized. Waiting for data...");
  }

private:
  // Type definitions for Sync
  using ImageMsg = sensor_msgs::msg::Image;
  using Float64StampedMsg = morai_msgs::msg::Float64Stamped;
  using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
    ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, Float64StampedMsg, Float64StampedMsg>;
  using Sync = message_filters::Synchronizer<MySyncPolicy>;

  void init_directories()
  {
    // Create base dir
    if (!fs::exists(save_path_)) {
      fs::create_directories(save_path_);
    }

    // Create meta dir
    fs::create_directories(save_path_ + "/meta");

    // Create camera dirs
    // cam0: front, cam1: front_left, cam4: front_right, cam2: rear_left, cam3: rear_right
    fs::create_directories(save_path_ + "/cam0"); // Front
    fs::create_directories(save_path_ + "/cam1"); // Front Left
    fs::create_directories(save_path_ + "/cam4"); // Front Right
    fs::create_directories(save_path_ + "/cam2"); // Rear Left
    fs::create_directories(save_path_ + "/cam3"); // Rear Right
  }

  void init_csv()
  {
    std::string csv_path = save_path_ + "/meta/meta.csv";
    bool file_exists = fs::exists(csv_path);

    csv_file_.open(csv_path, std::ios::out | std::ios::app);
    
    if (!file_exists) {
      // CSV Header matching prepare_data.py requirements
      // camX_filepath, ego_wheel_angle (steer), ego_accel (throttle), ego_brake, ego_vel_x, ego_vel_y
      csv_file_ << "timestamp,cam0_filepath,cam1_filepath,cam4_filepath,cam2_filepath,cam3_filepath," 
                << "ego_wheel_angle,ego_accel,ego_brake,ego_vel_x,ego_vel_y,ego_vel_z,ego_heading\n";
    }
  }

  void callback(
    const ImageMsg::ConstSharedPtr& msg0, // Front
    const ImageMsg::ConstSharedPtr& msg1, // Front Left
    const ImageMsg::ConstSharedPtr& msg4, // Front Right
    const ImageMsg::ConstSharedPtr& msg2, // Rear Left
    const ImageMsg::ConstSharedPtr& msg3, // Rear Right
    const Float64StampedMsg::ConstSharedPtr& velocity_msg,
    const Float64StampedMsg::ConstSharedPtr& steering_msg)
  {
    // Timestamp (use header stamp of the front camera)
    // Convert to nanoseconds
    uint64_t timestamp = msg0->header.stamp.sec * 1e9 + msg0->header.stamp.nanosec;
    std::string ts_str = std::to_string(timestamp);

    // Save Images
    // cam0
    std::string path0 = save_image(msg0, "cam0", ts_str);
    std::string path1 = save_image(msg1, "cam1", ts_str);
    std::string path4 = save_image(msg4, "cam4", ts_str);
    std::string path2 = save_image(msg2, "cam2", ts_str);
    std::string path3 = save_image(msg3, "cam3", ts_str);

    // Log progress periodically
    static int count = 0;
    if (count++ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Saved frame: %s | Vel: %.2f m/s", ts_str.c_str(), velocity_msg->data);
    }

    // Write to CSV
    // Note: pathX is absolute path, but prepare_data.py often expects relative to data_base_path
    // or we can store relative path. Let's store relative path "camX/timestamp.jpg"
    
    auto get_rel_path = [](const std::string& folder, const std::string& ts) {
      return folder + "/" + ts + ".jpg";
    };

    csv_file_ << ts_str << ","
              << get_rel_path("cam0", ts_str) << ","
              << get_rel_path("cam1", ts_str) << ","
              << get_rel_path("cam4", ts_str) << ","
              << get_rel_path("cam2", ts_str) << ","
              << get_rel_path("cam3", ts_str) << ","
              << steering_msg->data << ","
              << 0.0 << ","
              << 0.0 << ","
              << velocity_msg->data << ","
              << 0.0 << ","
              << 0.0 << ","
              << 0.0 << "\n";
              
    // Flush to ensure data is written
    csv_file_.flush();
  }

  std::string save_image(const ImageMsg::ConstSharedPtr& msg, const std::string& folder, const std::string& filename)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr;
      // Convert to BGR8 for OpenCV saving
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      std::string full_path = save_path_ + "/" + folder + "/" + filename + ".jpg";
      // Use high quality JPEG
      std::vector<int> compression_params;
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(95);
      
      cv::imwrite(full_path, cv_ptr->image, compression_params);
      
      return full_path;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return "";
    }
  }

  // Member variables
  std::string save_path_;
  std::ofstream csv_file_;

  message_filters::Subscriber<ImageMsg> cam0_sub_;
  message_filters::Subscriber<ImageMsg> cam1_sub_;
  message_filters::Subscriber<ImageMsg> cam4_sub_;
  message_filters::Subscriber<ImageMsg> cam2_sub_;
  message_filters::Subscriber<ImageMsg> cam3_sub_;
  message_filters::Subscriber<Float64StampedMsg> velocity_sub_;
  message_filters::Subscriber<Float64StampedMsg> steering_sub_;
  
  std::shared_ptr<Sync> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
