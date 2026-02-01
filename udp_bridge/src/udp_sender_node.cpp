#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

class UDPSenderNode : public rclcpp::Node {
public:
  UDPSenderNode()
  : rclcpp::Node("udp_sender") {
    declare_parameter<std::string>("udp.send_ip", "127.0.0.1");
    declare_parameter<int>("udp.ego_send_port", 9093);
    declare_parameter<int>("udp.npc_send_port", 9094);

    send_ip_ = get_parameter("udp.send_ip").as_string();
    ego_send_port_ = get_parameter("udp.ego_send_port").as_int();
    npc_send_port_ = get_parameter("udp.npc_send_port").as_int();

    setupSocket();
    setupSubscriptions();

    RCLCPP_INFO(get_logger(), "UDP Sender started (ego: %s:%d, npc: %s:%d)",
                send_ip_.c_str(), ego_send_port_, send_ip_.c_str(), npc_send_port_);
  }

  ~UDPSenderNode() override {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void setupSocket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Failed to create UDP send socket");
    }

    std::memset(&ego_send_addr_, 0, sizeof(ego_send_addr_));
    ego_send_addr_.sin_family = AF_INET;
    ego_send_addr_.sin_port = htons(static_cast<uint16_t>(ego_send_port_));
    ego_send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());

    std::memset(&npc_send_addr_, 0, sizeof(npc_send_addr_));
    npc_send_addr_.sin_family = AF_INET;
    npc_send_addr_.sin_port = htons(static_cast<uint16_t>(npc_send_port_));
    npc_send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());
  }

  void setupSubscriptions() {
    ego_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/ego/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::egoCmdCallback, this, std::placeholders::_1));

    npc_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/opponent/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::npcCmdCallback, this, std::placeholders::_1));
  }

  void egoCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::string payload = buildControlPayload(msg->linear.x, msg->linear.y, msg->angular.z);
    sendto(socket_fd_, payload.c_str(), payload.size(), 0,
           reinterpret_cast<sockaddr*>(&ego_send_addr_), sizeof(ego_send_addr_));
  }

  void npcCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::string payload = buildControlPayload(msg->linear.x, msg->linear.y, msg->angular.z);
    sendto(socket_fd_, payload.c_str(), payload.size(), 0,
           reinterpret_cast<sockaddr*>(&npc_send_addr_), sizeof(npc_send_addr_));
  }

  std::string buildControlPayload(double throttle, double brake, double steering) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "{"
       << "\"throttle\":" << throttle << ","
       << "\"brake\":" << brake << ","
       << "\"steering_wheel_angle\":" << std::setprecision(6) << steering
       << "}";
    return ss.str();
  }

  std::string send_ip_;
  int ego_send_port_ = 9093;
  int npc_send_port_ = 9094;

  int socket_fd_ = -1;
  sockaddr_in ego_send_addr_{};
  sockaddr_in npc_send_addr_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr npc_cmd_sub_;
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
