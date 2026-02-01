#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <morai_msgs/msg/ctrl_cmd.hpp>

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
    // UDP parameters
    declare_parameter<std::string>("udp.send_ip", "127.0.0.1");
    declare_parameter<int>("udp.ego_send_port", 9093);
    declare_parameter<int>("udp.npc_send_port", 9094);

    // Ego control parameters
    declare_parameter<double>("ego.throttle_kp", 0.1);
    declare_parameter<double>("ego.brake_kp", 0.1);
    declare_parameter<double>("ego.max_throttle", 1.0);
    declare_parameter<double>("ego.max_brake", 1.0);
    declare_parameter<double>("ego.steering_limit", 0.55);

    // NPC control parameters
    declare_parameter<double>("npc.throttle_kp", 0.1);
    declare_parameter<double>("npc.brake_kp", 0.1);
    declare_parameter<double>("npc.max_throttle", 1.0);
    declare_parameter<double>("npc.max_brake", 1.0);
    declare_parameter<double>("npc.steering_limit", 0.55);

    loadParameters();
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
  struct ControlConfig {
    double throttle_kp = 0.1;
    double brake_kp = 0.1;
    double max_throttle = 1.0;
    double max_brake = 1.0;
    double steering_limit = 0.55;
  };

  void loadParameters() {
    send_ip_ = get_parameter("udp.send_ip").as_string();
    ego_send_port_ = get_parameter("udp.ego_send_port").as_int();
    npc_send_port_ = get_parameter("udp.npc_send_port").as_int();

    // Ego control config
    ego_config_.throttle_kp = get_parameter("ego.throttle_kp").as_double();
    ego_config_.brake_kp = get_parameter("ego.brake_kp").as_double();
    ego_config_.max_throttle = get_parameter("ego.max_throttle").as_double();
    ego_config_.max_brake = get_parameter("ego.max_brake").as_double();
    ego_config_.steering_limit = get_parameter("ego.steering_limit").as_double();

    // NPC control config
    npc_config_.throttle_kp = get_parameter("npc.throttle_kp").as_double();
    npc_config_.brake_kp = get_parameter("npc.brake_kp").as_double();
    npc_config_.max_throttle = get_parameter("npc.max_throttle").as_double();
    npc_config_.max_brake = get_parameter("npc.max_brake").as_double();
    npc_config_.steering_limit = get_parameter("npc.steering_limit").as_double();
  }

  void setupSocket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      throw std::runtime_error("Failed to create UDP send socket");
    }

    // Ego send address
    std::memset(&ego_send_addr_, 0, sizeof(ego_send_addr_));
    ego_send_addr_.sin_family = AF_INET;
    ego_send_addr_.sin_port = htons(static_cast<uint16_t>(ego_send_port_));
    ego_send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());

    // NPC send address
    std::memset(&npc_send_addr_, 0, sizeof(npc_send_addr_));
    npc_send_addr_.sin_family = AF_INET;
    npc_send_addr_.sin_port = htons(static_cast<uint16_t>(npc_send_port_));
    npc_send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());
  }

  void setupSubscriptions() {
    // Ego subscriptions
    ego_cmd_sub_ = create_subscription<morai_msgs::msg::CtrlCmd>(
      "/ego/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::egoCmdCallback, this, std::placeholders::_1));

    ego_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/ego/vehicle/velocity", 10,
      std::bind(&UDPSenderNode::egoVelocityCallback, this, std::placeholders::_1));

    // NPC subscriptions
    npc_cmd_sub_ = create_subscription<morai_msgs::msg::CtrlCmd>(
      "/opponent/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::npcCmdCallback, this, std::placeholders::_1));

    npc_vel_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/opponent/vehicle/velocity", 10,
      std::bind(&UDPSenderNode::npcVelocityCallback, this, std::placeholders::_1));
  }

  void egoVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    ego_current_speed_ = msg->data;
  }

  void npcVelocityCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    npc_current_speed_ = msg->data;
  }

  void egoCmdCallback(const morai_msgs::msg::CtrlCmd::SharedPtr msg) {
    std::string payload = buildControlPayload(msg->throttle, msg->brake, msg->steering_wheel_angle);
    sendto(socket_fd_, payload.c_str(), payload.size(), 0,
           reinterpret_cast<sockaddr*>(&ego_send_addr_), sizeof(ego_send_addr_));
  }

  void npcCmdCallback(const morai_msgs::msg::CtrlCmd::SharedPtr msg) {
    std::string payload = buildControlPayload(msg->throttle, msg->brake, msg->steering_wheel_angle);
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

  ControlConfig ego_config_;
  ControlConfig npc_config_;

  double ego_current_speed_ = 0.0;
  double npc_current_speed_ = 0.0;

  int socket_fd_ = -1;
  sockaddr_in ego_send_addr_{};
  sockaddr_in npc_send_addr_{};

  rclcpp::Subscription<morai_msgs::msg::CtrlCmd>::SharedPtr ego_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_vel_sub_;
  rclcpp::Subscription<morai_msgs::msg::CtrlCmd>::SharedPtr npc_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr npc_vel_sub_;
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
