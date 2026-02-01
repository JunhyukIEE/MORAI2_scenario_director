#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <cstring>
#include <sstream>
#include <string>

class UDPSenderNode : public rclcpp::Node {
public:
  UDPSenderNode()
  : rclcpp::Node("udp_sender") {
    declare_parameter<std::string>("udp.send_ip", "127.0.0.1");
    declare_parameter<int>("udp.send_port", 9093);
    declare_parameter<double>("control.throttle_kp", 0.1);
    declare_parameter<double>("control.brake_kp", 0.1);
    declare_parameter<double>("control.max_throttle", 1.0);
    declare_parameter<double>("control.max_brake", 1.0);
    declare_parameter<double>("control.steering_limit", 0.55);

    send_ip_ = get_parameter("udp.send_ip").as_string();
    send_port_ = get_parameter("udp.send_port").as_int();
    throttle_kp_ = get_parameter("control.throttle_kp").as_double();
    brake_kp_ = get_parameter("control.brake_kp").as_double();
    max_throttle_ = get_parameter("control.max_throttle").as_double();
    max_brake_ = get_parameter("control.max_brake").as_double();
    steering_limit_ = get_parameter("control.steering_limit").as_double();

    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Failed to create UDP send socket");
    }

    std::memset(&send_addr_, 0, sizeof(send_addr_));
    send_addr_.sin_family = AF_INET;
    send_addr_.sin_port = htons(static_cast<uint16_t>(send_port_));
    send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/ego/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::cmd_callback, this, std::placeholders::_1));

    vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10,
      std::bind(&UDPSenderNode::velocity_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "UDP Sender started (%s:%d)", send_ip_.c_str(), send_port_);
  }

  ~UDPSenderNode() override {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_speed_ = msg->data;
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double target_speed = msg->linear.x;
    double steering = msg->angular.z;

    steering = std::max(-steering_limit_, std::min(steering_limit_, steering));

    const double speed_error = target_speed - current_speed_;
    double throttle = 0.0;
    double brake = 0.0;

    if (speed_error > 0.0) {
      throttle = std::min(max_throttle_, throttle_kp_ * speed_error);
    } else {
      brake = std::min(max_brake_, brake_kp_ * std::abs(speed_error));
    }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "{"
       << "\"throttle\":" << throttle << ","
       << "\"brake\":" << brake << ","
       << "\"steering\":" << std::setprecision(6) << steering
       << "}";

    const std::string payload = ss.str();
    sendto(socket_fd_, payload.c_str(), payload.size(), 0,
           reinterpret_cast<sockaddr*>(&send_addr_), sizeof(send_addr_));
  }

  std::string send_ip_;
  int send_port_ = 9093;

  double throttle_kp_ = 0.1;
  double brake_kp_ = 0.1;
  double max_throttle_ = 1.0;
  double max_brake_ = 1.0;
  double steering_limit_ = 0.55;

  double current_speed_ = 0.0;

  int socket_fd_ = -1;
  sockaddr_in send_addr_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vel_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UDPSenderNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "UDPSenderNode error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
