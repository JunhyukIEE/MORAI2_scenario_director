#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cctype>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

namespace {

constexpr int NUM_NPCS = 9;

bool extract_number(const std::string &json, const std::string &key, double &out) {
  const std::string token = "\"" + key + "\"";
  size_t pos = json.find(token);
  if (pos == std::string::npos) {
    return false;
  }
  pos = json.find(':', pos + token.size());
  if (pos == std::string::npos) {
    return false;
  }
  pos += 1;
  while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
    ++pos;
  }
  const char *start = json.c_str() + pos;
  char *end = nullptr;
  double value = std::strtod(start, &end);
  if (end == start) {
    return false;
  }
  out = value;
  return true;
}

std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  std::array<double, 4> q{};
  q[0] = sr * cp * cy - cr * sp * sy; // x
  q[1] = cr * sp * cy + sr * cp * sy; // y
  q[2] = cr * cp * sy - sr * sp * cy; // z
  q[3] = cr * cp * cy + sr * sp * sy; // w
  return q;
}

}  // namespace

class UDPReceiverNode : public rclcpp::Node {
public:
  UDPReceiverNode()
  : rclcpp::Node("udp_receiver"), running_(true) {
    declare_parameter<std::string>("udp.recv_ip", "0.0.0.0");
    declare_parameter<int>("udp.ego_recv_port", 9090);
    declare_parameter<bool>("udp.angles_in_degrees", true);

    // Declare NPC ports (9190, 9290, ..., 9990)
    for (int i = 1; i <= NUM_NPCS; ++i) {
      std::string param_name = "udp.npc" + std::to_string(i) + "_recv_port";
      int default_port = 9000 + i * 100 + 90;  // 9190, 9290, ..., 9990
      declare_parameter<int>(param_name, default_port);
    }

    recv_ip_ = get_parameter("udp.recv_ip").as_string();
    ego_recv_port_ = get_parameter("udp.ego_recv_port").as_int();
    angles_in_degrees_ = get_parameter("udp.angles_in_degrees").as_bool();

    // Get NPC ports
    for (int i = 1; i <= NUM_NPCS; ++i) {
      std::string param_name = "udp.npc" + std::to_string(i) + "_recv_port";
      npc_recv_ports_[i-1] = get_parameter(param_name).as_int();
    }

    // Ego publishers
    ego_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ego/odom", 10);
    ego_velocity_pub_ = create_publisher<std_msgs::msg::Float64>("/ego/vehicle/velocity", 10);
    ego_accel_pub_ = create_publisher<std_msgs::msg::Float64>("/ego/vehicle/acceleration", 10);
    ego_steering_pub_ = create_publisher<std_msgs::msg::Float64>("/ego/vehicle/steering_state", 10);

    // NPC publishers
    for (int i = 1; i <= NUM_NPCS; ++i) {
      std::string prefix = "/NPC_" + std::to_string(i);
      npc_odom_pubs_[i-1] = create_publisher<nav_msgs::msg::Odometry>(prefix + "/odom", 10);
      npc_velocity_pubs_[i-1] = create_publisher<std_msgs::msg::Float64>(prefix + "/vehicle/velocity", 10);
      npc_accel_pubs_[i-1] = create_publisher<std_msgs::msg::Float64>(prefix + "/vehicle/acceleration", 10);
      npc_steering_pubs_[i-1] = create_publisher<std_msgs::msg::Float64>(prefix + "/vehicle/steering_state", 10);
    }

    setup_sockets();

    // Start ego thread
    ego_thread_ = std::thread(&UDPReceiverNode::ego_receive_loop, this);

    // Start NPC threads
    for (int i = 0; i < NUM_NPCS; ++i) {
      npc_threads_[i] = std::thread(&UDPReceiverNode::npc_receive_loop, this, i);
    }

    RCLCPP_INFO(get_logger(), "UDP Receiver started (ego: %d, NPCs: 9190-9990)",
                ego_recv_port_);
  }

  ~UDPReceiverNode() override {
    running_.store(false);
    if (ego_thread_.joinable()) {
      ego_thread_.join();
    }
    for (int i = 0; i < NUM_NPCS; ++i) {
      if (npc_threads_[i].joinable()) {
        npc_threads_[i].join();
      }
    }
    if (ego_sock_ >= 0) {
      close(ego_sock_);
    }
    for (int i = 0; i < NUM_NPCS; ++i) {
      if (npc_socks_[i] >= 0) {
        close(npc_socks_[i]);
      }
    }
  }

private:
  void setup_sockets() {
    ego_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (ego_sock_ < 0) {
      throw std::runtime_error("Failed to create ego UDP socket");
    }

    int reuse = 1;
    setsockopt(ego_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // 수신 버퍼 크기 증가 (8MB) - 패킷 손실 방지
    int rcvbuf_size = 8 * 1024 * 1024;
    setsockopt(ego_sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));

    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms timeout
    setsockopt(ego_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in ego_addr{};
    ego_addr.sin_family = AF_INET;
    ego_addr.sin_port = htons(static_cast<uint16_t>(ego_recv_port_));
    ego_addr.sin_addr.s_addr = inet_addr(recv_ip_.c_str());

    if (bind(ego_sock_, reinterpret_cast<sockaddr*>(&ego_addr), sizeof(ego_addr)) < 0) {
      throw std::runtime_error("Failed to bind ego UDP socket");
    }

    // Setup NPC sockets
    for (int i = 0; i < NUM_NPCS; ++i) {
      npc_socks_[i] = socket(AF_INET, SOCK_DGRAM, 0);
      if (npc_socks_[i] < 0) {
        throw std::runtime_error("Failed to create NPC_" + std::to_string(i+1) + " UDP socket");
      }

      setsockopt(npc_socks_[i], SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
      setsockopt(npc_socks_[i], SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));
      setsockopt(npc_socks_[i], SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

      sockaddr_in npc_addr{};
      npc_addr.sin_family = AF_INET;
      npc_addr.sin_port = htons(static_cast<uint16_t>(npc_recv_ports_[i]));
      npc_addr.sin_addr.s_addr = inet_addr(recv_ip_.c_str());

      if (bind(npc_socks_[i], reinterpret_cast<sockaddr*>(&npc_addr), sizeof(npc_addr)) < 0) {
        throw std::runtime_error("Failed to bind NPC_" + std::to_string(i+1) + " UDP socket (port " +
                                 std::to_string(npc_recv_ports_[i]) + ")");
      }
    }
  }

  void ego_receive_loop() {
    receive_loop(ego_sock_, -1);  // -1 indicates ego
  }

  void npc_receive_loop(int npc_index) {
    receive_loop(npc_socks_[npc_index], npc_index);
  }

  void receive_loop(int sock, int npc_index) {
    // npc_index: -1 for ego, 0-8 for NPC_1 to NPC_9
    while (running_.load()) {
      char buffer[4096];
      sockaddr_in sender{};
      socklen_t sender_len = sizeof(sender);
      const ssize_t len = recvfrom(sock, buffer, sizeof(buffer), 0,
                                   reinterpret_cast<sockaddr*>(&sender), &sender_len);
      if (len <= 0) {
        continue;
      }

      constexpr ssize_t kExpectedPacketLen = 108;
      if (len == kExpectedPacketLen) {
        parse_binary_vehicle(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(len), npc_index);
        continue;
      }

      std::string data(buffer, static_cast<size_t>(len));
      if (!parse_json_vehicle(data, npc_index)) {
        parse_binary_vehicle(reinterpret_cast<const uint8_t*>(buffer), static_cast<size_t>(len), npc_index);
      }
    }
  }

  bool parse_json_vehicle(const std::string &data, int npc_index) {
    // Quick sanity check
    if (data.find('{') == std::string::npos) {
      return false;
    }

    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double vx = 0.0, vy = 0.0, ax = 0.0, ay = 0.0, steering = 0.0;

    extract_number(data, "x", x);
    extract_number(data, "y", y);
    extract_number(data, "z", z);
    extract_number(data, "roll", roll);
    extract_number(data, "pitch", pitch);
    extract_number(data, "yaw", yaw);
    extract_number(data, "vx", vx);
    extract_number(data, "vy", vy);
    extract_number(data, "ax", ax);
    extract_number(data, "ay", ay);
    extract_number(data, "steering", steering);

    if (angles_in_degrees_) {
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      roll *= kDegToRad;
      pitch *= kDegToRad;
      yaw *= kDegToRad;
    }

    publish_vehicle(x, y, z, roll, pitch, yaw, vx, vy, ax, ay, steering, npc_index);
    return true;
  }

  void parse_binary_vehicle(const uint8_t *data, size_t len, int npc_index) {
    constexpr size_t HEADER_SIZE = 36;
    constexpr size_t NUM_FLOATS = 18;
    constexpr size_t EXPECTED_SIZE = HEADER_SIZE + NUM_FLOATS * sizeof(float);

    if (len < EXPECTED_SIZE) {
      return;
    }

    const uint8_t *payload = data + HEADER_SIZE;
    float fvalues[NUM_FLOATS];
    std::memcpy(fvalues, payload, NUM_FLOATS * sizeof(float));

    double x = fvalues[0];
    double y = fvalues[1];
    double z = fvalues[2];
    double roll = fvalues[3];
    double pitch = fvalues[4];
    double yaw = fvalues[5];
    double vx = fvalues[6];
    double vy = fvalues[7];
    double ax = fvalues[9];
    double ay = fvalues[10];
    double steering = fvalues[17];

    if (angles_in_degrees_) {
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      roll *= kDegToRad;
      pitch *= kDegToRad;
      yaw *= kDegToRad;
    }

    publish_vehicle(x, y, z, roll, pitch, yaw, vx, vy, ax, ay, steering, npc_index);
  }

  void publish_vehicle(double x, double y, double z,
                       double roll, double pitch, double yaw,
                       double vx, double vy, double ax, double ay, double steering,
                       int npc_index) {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now();
    odom.header.frame_id = "world";

    if (npc_index < 0) {
      odom.child_frame_id = "ego_base_link";
    } else {
      odom.child_frame_id = "npc_" + std::to_string(npc_index + 1) + "_base_link";
    }

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;

    auto q = euler_to_quaternion(roll, pitch, yaw);
    odom.pose.pose.orientation.x = q[0];
    odom.pose.pose.orientation.y = q[1];
    odom.pose.pose.orientation.z = q[2];
    odom.pose.pose.orientation.w = q[3];

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    std_msgs::msg::Float64 vel_msg;
    vel_msg.data = std::sqrt(vx * vx + vy * vy);

    std_msgs::msg::Float64 accel_msg;
    accel_msg.data = std::sqrt(ax * ax + ay * ay);

    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = steering;

    if (npc_index < 0) {
      // Ego
      ego_odom_pub_->publish(odom);
      ego_velocity_pub_->publish(vel_msg);
      ego_accel_pub_->publish(accel_msg);
      ego_steering_pub_->publish(steer_msg);
    } else {
      // NPC
      npc_odom_pubs_[npc_index]->publish(odom);
      npc_velocity_pubs_[npc_index]->publish(vel_msg);
      npc_accel_pubs_[npc_index]->publish(accel_msg);
      npc_steering_pubs_[npc_index]->publish(steer_msg);
    }
  }

  std::string recv_ip_;
  int ego_recv_port_ = 9090;
  std::array<int, NUM_NPCS> npc_recv_ports_;

  int ego_sock_ = -1;
  std::array<int, NUM_NPCS> npc_socks_;
  bool angles_in_degrees_ = true;

  std::atomic<bool> running_;
  std::thread ego_thread_;
  std::array<std::thread, NUM_NPCS> npc_threads_;

  // Ego publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ego_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ego_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ego_accel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ego_steering_pub_;

  // NPC publishers
  std::array<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr, NUM_NPCS> npc_odom_pubs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_NPCS> npc_velocity_pubs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_NPCS> npc_accel_pubs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, NUM_NPCS> npc_steering_pubs_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<UDPReceiverNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    fprintf(stderr, "UDPReceiverNode error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
