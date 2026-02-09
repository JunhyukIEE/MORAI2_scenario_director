#include <rclcpp/rclcpp.hpp>
#include "scenario_director/msg/vehicle_cmd.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstring>
#include <functional>
#include <string>

namespace {
constexpr int NUM_NPCS = 9;
}

class UDPSenderNode : public rclcpp::Node {
public:
  UDPSenderNode()
  : rclcpp::Node("udp_sender") {
    declare_parameter<std::string>("udp.send_ip", "127.0.0.1");
    declare_parameter<int>("udp.ego_send_port", 9091);

    // Declare NPC send ports (9191, 9291, ..., 9991)
    for (int i = 1; i <= NUM_NPCS; ++i) {
      std::string param_name = "udp.npc" + std::to_string(i) + "_send_port";
      int default_port = 9000 + i * 100 + 91;  // 9191, 9291, ..., 9991
      declare_parameter<int>(param_name, default_port);
    }

    send_ip_ = get_parameter("udp.send_ip").as_string();
    ego_send_port_ = get_parameter("udp.ego_send_port").as_int();

    // Get NPC ports
    for (int i = 1; i <= NUM_NPCS; ++i) {
      std::string param_name = "udp.npc" + std::to_string(i) + "_send_port";
      npc_send_ports_[i-1] = get_parameter(param_name).as_int();
    }

    setupSocket();
    setupSubscriptions();

    RCLCPP_INFO(get_logger(), "UDP Sender started (ego: %s:%d)", send_ip_.c_str(), ego_send_port_);
    for (int i = 0; i < NUM_NPCS; ++i) {
      RCLCPP_INFO(get_logger(), "  NPC_%d send port: %d", i + 1, npc_send_ports_[i]);
    }
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

    // Setup ego send address
    std::memset(&ego_send_addr_, 0, sizeof(ego_send_addr_));
    ego_send_addr_.sin_family = AF_INET;
    ego_send_addr_.sin_port = htons(static_cast<uint16_t>(ego_send_port_));
    ego_send_addr_.sin_addr.s_addr = inet_addr(send_ip_.c_str());

    // Setup NPC send addresses
    for (int i = 0; i < NUM_NPCS; ++i) {
      std::memset(&npc_send_addrs_[i], 0, sizeof(npc_send_addrs_[i]));
      npc_send_addrs_[i].sin_family = AF_INET;
      npc_send_addrs_[i].sin_port = htons(static_cast<uint16_t>(npc_send_ports_[i]));
      npc_send_addrs_[i].sin_addr.s_addr = inet_addr(send_ip_.c_str());
    }
  }

  void setupSubscriptions() {
    // Ego subscription
    ego_cmd_sub_ = create_subscription<scenario_director::msg::VehicleCmd>(
      "/ego/ctrl_cmd", 10,
      std::bind(&UDPSenderNode::egoCmdCallback, this, std::placeholders::_1));

    // NPC subscriptions
    for (int i = 0; i < NUM_NPCS; ++i) {
      std::string topic = "/NPC_" + std::to_string(i + 1) + "/ctrl_cmd";
      npc_cmd_subs_[i] = create_subscription<scenario_director::msg::VehicleCmd>(
        topic, 10,
        [this, i](const scenario_director::msg::VehicleCmd::SharedPtr msg) {
          npcCmdCallback(msg, i);
        });
    }
  }

  void egoCmdCallback(const scenario_director::msg::VehicleCmd::SharedPtr msg) {
    constexpr double RAD_TO_DEG = 57.2957795131;

    double throttle = msg->throttle;
    double brake = msg->brake;
    double steering_deg = msg->steering * RAD_TO_DEG;

    uint8_t payload[24];  // 3 doubles = 24 bytes
    std::memcpy(payload, &throttle, sizeof(double));
    std::memcpy(payload + 8, &brake, sizeof(double));
    std::memcpy(payload + 16, &steering_deg, sizeof(double));

    ssize_t sent = sendto(socket_fd_, payload, sizeof(payload), 0,
           reinterpret_cast<sockaddr*>(&ego_send_addr_), sizeof(ego_send_addr_));

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Ego cmd sent: throttle=%.2f, brake=%.2f, steer=%.2f, bytes=%zd",
                          throttle, brake, steering_deg, sent);
  }

  void npcCmdCallback(const scenario_director::msg::VehicleCmd::SharedPtr msg, int npc_index) {
    constexpr double RAD_TO_DEG = 57.2957795131;

    double throttle = msg->throttle;
    double brake = msg->brake;
    double steering_deg = msg->steering * RAD_TO_DEG;

    uint8_t payload[24];
    std::memcpy(payload, &throttle, sizeof(double));
    std::memcpy(payload + 8, &brake, sizeof(double));
    std::memcpy(payload + 16, &steering_deg, sizeof(double));

    ssize_t sent = sendto(socket_fd_, payload, sizeof(payload), 0,
           reinterpret_cast<sockaddr*>(&npc_send_addrs_[npc_index]), sizeof(npc_send_addrs_[npc_index]));

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "NPC_%d cmd sent to port %d: throttle=%.2f, brake=%.2f, steer=%.2f, bytes=%zd",
                          npc_index + 1, npc_send_ports_[npc_index],
                          throttle, brake, steering_deg, sent);
  }

  std::string send_ip_;
  int ego_send_port_ = 9091;
  std::array<int, NUM_NPCS> npc_send_ports_;

  int socket_fd_ = -1;
  sockaddr_in ego_send_addr_{};
  std::array<sockaddr_in, NUM_NPCS> npc_send_addrs_;

  rclcpp::Subscription<scenario_director::msg::VehicleCmd>::SharedPtr ego_cmd_sub_;
  std::array<rclcpp::Subscription<scenario_director::msg::VehicleCmd>::SharedPtr, NUM_NPCS> npc_cmd_subs_;
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
