#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

class CameraReceiverNode : public rclcpp::Node {
public:
  CameraReceiverNode()
  : rclcpp::Node("camera_receiver"), running_(true) {
    declareParameters();
    loadParameters();
    setupPublishers();
    setupSockets();
    startThreads();

    // Startup logging intentionally suppressed.
  }

  ~CameraReceiverNode() override {
    running_.store(false);
    for (auto &t : threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    for (int sock : sockets_) {
      if (sock >= 0) {
        close(sock);
      }
    }
  }

private:
  struct CameraConfig {
    std::string name;
    int port;
    std::string topic;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
  };

  void declareParameters() {
    declare_parameter<std::string>("udp.recv_ip", "0.0.0.0");

    // Camera ports
    declare_parameter<int>("camera.front.port", 6666);
    declare_parameter<int>("camera.left_front.port", 7700);
    declare_parameter<int>("camera.left_rear.port", 7755);
    declare_parameter<int>("camera.right_front.port", 8800);
    declare_parameter<int>("camera.right_rear.port", 8855);

    // Camera enable flags
    declare_parameter<bool>("camera.front.enable", true);
    declare_parameter<bool>("camera.left_front.enable", true);
    declare_parameter<bool>("camera.left_rear.enable", true);
    declare_parameter<bool>("camera.right_front.enable", true);
    declare_parameter<bool>("camera.right_rear.enable", true);
  }

  void loadParameters() {
    recv_ip_ = get_parameter("udp.recv_ip").as_string();

    // Define camera configurations
    std::vector<std::tuple<std::string, std::string, std::string>> camera_defs = {
      {"front", "camera.front", "/sensing/camera/front/image_raw"},
      {"left_front", "camera.left_front", "/sensing/camera/left_front/image_raw"},
      {"left_rear", "camera.left_rear", "/sensing/camera/left_rear/image_raw"},
      {"right_front", "camera.right_front", "/sensing/camera/right_front/image_raw"},
      {"right_rear", "camera.right_rear", "/sensing/camera/right_rear/image_raw"}
    };

    for (const auto &[name, param_prefix, topic] : camera_defs) {
      bool enabled = get_parameter(param_prefix + ".enable").as_bool();
      if (enabled) {
        CameraConfig cfg;
        cfg.name = name;
        cfg.port = get_parameter(param_prefix + ".port").as_int();
        cfg.topic = topic;
        camera_configs_.push_back(cfg);
        // Per-camera enable logging intentionally suppressed.
      }
    }
  }

  void setupPublishers() {
    for (auto &cfg : camera_configs_) {
      cfg.publisher = create_publisher<sensor_msgs::msg::Image>(cfg.topic, 10);
    }
  }

  void setupSockets() {
    for (const auto &cfg : camera_configs_) {
      int sock = socket(AF_INET, SOCK_DGRAM, 0);
      if (sock < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to create socket for camera '%s'", cfg.name.c_str());
        sockets_.push_back(-1);
        continue;
      }

      int reuse = 1;
      setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

      // Increase receive buffer for large images
      int recv_buf_size = 10 * 1024 * 1024; // 10MB
      setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));

      timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 100000; // 100ms timeout
      setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(static_cast<uint16_t>(cfg.port));
      addr.sin_addr.s_addr = inet_addr(recv_ip_.c_str());

      if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to bind socket for camera '%s' on port %d",
                     cfg.name.c_str(), cfg.port);
        close(sock);
        sockets_.push_back(-1);
        continue;
      }

      sockets_.push_back(sock);
    }
  }

  void startThreads() {
    for (size_t i = 0; i < camera_configs_.size(); ++i) {
      if (sockets_[i] >= 0) {
        threads_.emplace_back(&CameraReceiverNode::receiveLoop, this, i);
      }
    }
  }

  void receiveLoop(size_t camera_idx) {
    const auto &cfg = camera_configs_[camera_idx];
    int sock = sockets_[camera_idx];

    // Buffer for receiving image data (max ~10MB for large images)
    std::vector<uint8_t> buffer(10 * 1024 * 1024);

    while (running_.load()) {
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);

      ssize_t len = recvfrom(sock, buffer.data(), buffer.size(), 0,
                             reinterpret_cast<sockaddr*>(&sender), &sender_len);

      if (len <= 0) {
        continue;
      }

      processImage(cfg, buffer.data(), static_cast<size_t>(len));
    }
  }

  void processImage(const CameraConfig &cfg, const uint8_t *data, size_t len) {
    try {
      // Check for MORAI format: 4-byte header + JPEG data
      // Header format: [size (4 bytes little-endian)] [JPEG data starting with FF D8]
      const uint8_t *img_start = data;
      size_t img_len = len;

      if (len > 4 && data[4] == 0xFF && data[5] == 0xD8) {
        // MORAI format detected: skip 4-byte header
        img_start = data + 4;
        img_len = len - 4;
      }

      // Try to decode as JPEG/PNG
      std::vector<uint8_t> img_data(img_start, img_start + img_len);
      cv::Mat image = cv::imdecode(img_data, cv::IMREAD_COLOR);

      if (image.empty()) {
        // Maybe raw image data - try to interpret as raw BGR
        if (len == 320 * 240 * 3) {
          image = cv::Mat(240, 320, CV_8UC3, const_cast<uint8_t*>(data)).clone();
        } else if (len == 640 * 480 * 3) {
          image = cv::Mat(480, 640, CV_8UC3, const_cast<uint8_t*>(data)).clone();
        } else if (len == 1280 * 720 * 3) {
          image = cv::Mat(720, 1280, CV_8UC3, const_cast<uint8_t*>(data)).clone();
        } else if (len == 1920 * 1080 * 3) {
          image = cv::Mat(1080, 1920, CV_8UC3, const_cast<uint8_t*>(data)).clone();
        } else {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Camera '%s': Unknown image format (size: %zu bytes)",
                               cfg.name.c_str(), len);
          return;
        }
      }

      // Convert to ROS message
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
      msg->header.stamp = now();
      msg->header.frame_id = cfg.name + "_camera_link";

      cfg.publisher->publish(*msg);

    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Camera '%s': Failed to process image: %s",
                           cfg.name.c_str(), e.what());
    }
  }

  std::string recv_ip_;
  std::vector<CameraConfig> camera_configs_;
  std::vector<int> sockets_;
  std::vector<std::thread> threads_;
  std::atomic<bool> running_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CameraReceiverNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "CameraReceiverNode error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
