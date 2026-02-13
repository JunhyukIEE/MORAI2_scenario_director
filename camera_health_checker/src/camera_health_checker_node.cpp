#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;
using namespace std::chrono_literals;

class CameraHealthChecker : public rclcpp::Node
{
public:
  CameraHealthChecker()
  : Node("camera_health_checker")
  {
    topic_names_ = declare_parameter<std::vector<std::string>>(
      "topic_names",
      {
        "/sensing/camera/front/image_raw",
        "/sensing/camera/front_left/image_raw",
        "/sensing/camera/rear_left/image_raw",
        "/sensing/camera/front_right/image_raw",
        "/sensing/camera/rear_right/image_raw"
      });

    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics_camera");
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 30.0);
    min_fps_ = declare_parameter<double>("min_fps", 15.0);
    window_size_ = declare_parameter<int>("rate_window_size", 30);
    use_approx_sync_ = declare_parameter<bool>("use_approx_sync", true);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 1.0);
    status_period_sec_ = declare_parameter<double>("status_period_sec", 1.0);

    if (topic_names_.size() != 5) {
      RCLCPP_WARN(get_logger(), "Expected 5 topics, got %zu", topic_names_.size());
    }

    if (min_fps_ > 0.0 && stale_timeout_sec_ <= 0.0) {
      stale_timeout_sec_ = 2.0 / min_fps_;
    }

    states_.resize(topic_names_.size());
    for (size_t i = 0; i < topic_names_.size(); ++i) {
      states_[i].name = topic_names_[i];
    }

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::SystemDefaultsQoS());

    setup_rate_subscriptions();
    setup_sync();

    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_period_sec_),
      std::bind(&CameraHealthChecker::publish_status, this));
  }

private:
  struct TopicState
  {
    std::string name;
    bool have_stamp{false};
    bool have_arrival{false};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_arrival{0, 0, RCL_STEADY_TIME};
    std::deque<double> intervals_sec;
    double fps{0.0};
  };

  void setup_rate_subscriptions()
  {
    rclcpp::SensorDataQoS qos;
    for (size_t i = 0; i < topic_names_.size(); ++i) {
      auto sub = create_subscription<Image>(
        topic_names_[i], qos,
        [this, i](Image::ConstSharedPtr msg) { this->on_image_rate(i, msg); });
      rate_subs_.push_back(sub);
    }
  }

  void setup_sync()
  {
    rclcpp::SensorDataQoS qos;
    subs_.reserve(topic_names_.size());
    for (const auto & name : topic_names_) {
      subs_.emplace_back(std::make_shared<message_filters::Subscriber<Image>>(this, name, qos.get_rmw_qos_profile()));
    }

    if (topic_names_.size() != 5) {
      return;
    }

    if (use_approx_sync_) {
      using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image, Image>;
      approx_sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(10),
        *subs_[0], *subs_[1], *subs_[2], *subs_[3], *subs_[4]);
      approx_sync_->registerCallback(
        std::bind(&CameraHealthChecker::on_sync, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
    } else {
      using Policy = message_filters::sync_policies::ExactTime<Image, Image, Image, Image, Image>;
      exact_sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(10),
        *subs_[0], *subs_[1], *subs_[2], *subs_[3], *subs_[4]);
      exact_sync_->registerCallback(
        std::bind(&CameraHealthChecker::on_sync, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));
    }
  }

  void on_image_rate(size_t index, const Image::ConstSharedPtr & msg)
  {
    if (index >= states_.size()) {
      return;
    }

    auto now = this->get_clock()->now();
    auto & state = states_[index];

    rclcpp::Time msg_stamp(msg->header.stamp);
    if (state.have_stamp) {
      auto dt = (msg_stamp - state.last_stamp).seconds();
      if (dt > 0.0) {
        state.intervals_sec.push_back(dt);
        while (state.intervals_sec.size() > static_cast<size_t>(window_size_)) {
          state.intervals_sec.pop_front();
        }
        double sum = 0.0;
        for (double v : state.intervals_sec) {
          sum += v;
        }
        if (!state.intervals_sec.empty()) {
          state.fps = static_cast<double>(state.intervals_sec.size()) / sum;
        }
      }
    }

    state.have_stamp = true;
    state.have_arrival = true;
    state.last_stamp = msg_stamp;
    state.last_arrival = now;
  }

  void on_sync(const Image::ConstSharedPtr & a,
               const Image::ConstSharedPtr & b,
               const Image::ConstSharedPtr & c,
               const Image::ConstSharedPtr & d,
               const Image::ConstSharedPtr & e)
  {
    std::array<rclcpp::Time, 5> stamps = {
      rclcpp::Time(a->header.stamp),
      rclcpp::Time(b->header.stamp),
      rclcpp::Time(c->header.stamp),
      rclcpp::Time(d->header.stamp),
      rclcpp::Time(e->header.stamp)
    };

    auto min_it = std::min_element(stamps.begin(), stamps.end());
    auto max_it = std::max_element(stamps.begin(), stamps.end());
    last_sync_delta_ms_ = (*max_it - *min_it).seconds() * 1000.0;
    last_sync_stamp_ = this->get_clock()->now();
    have_sync_ = true;
  }

  void publish_status()
  {
    diagnostic_msgs::msg::DiagnosticArray array_msg;
    array_msg.header.stamp = this->get_clock()->now();

    diagnostic_msgs::msg::DiagnosticStatus status_msg;
    status_msg.name = "camera_sync_health";
    status_msg.hardware_id = "camera_array";
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_msg.message = "OK";

    auto now = this->get_clock()->now();

    if (topic_names_.size() != 5) {
      set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "topic count mismatch");
    }

    if (have_sync_) {
      add_kv(status_msg, "sync_delta_ms", last_sync_delta_ms_);
      if (last_sync_delta_ms_ > sync_tolerance_ms_ * 2.0) {
        set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "sync delta too large");
      } else if (last_sync_delta_ms_ > sync_tolerance_ms_) {
        set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "sync delta high");
      }
    } else {
      set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "no synchronized messages yet");
    }

    for (const auto & state : states_) {
      std::string base = state.name;
      if (!state.have_arrival) {
        set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "missing data");
        continue;
      }

      double age = (now - state.last_arrival).seconds();
      add_kv(status_msg, base + ".age_sec", age);
      add_kv(status_msg, base + ".fps", state.fps);

      if (age > stale_timeout_sec_ * 2.0) {
        set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          "stale data");
      } else if (age > stale_timeout_sec_) {
        set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "stale data");
      }

      if (min_fps_ > 0.0) {
        if (state.fps < min_fps_ * 0.5) {
          set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            "fps too low");
        } else if (state.fps < min_fps_) {
          set_status(status_msg, diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "fps low");
        }
      }
    }

    array_msg.status.push_back(status_msg);
    diagnostics_pub_->publish(array_msg);

    double min_fps = std::numeric_limits<double>::infinity();
    double max_fps = 0.0;
    std::string per_cam_fps;
    for (const auto & state : states_) {
      if (state.have_arrival) {
        min_fps = std::min(min_fps, state.fps);
        max_fps = std::max(max_fps, state.fps);
        if (!per_cam_fps.empty()) {
          per_cam_fps += ", ";
        }
        per_cam_fps += short_name(state.name) + "=" + format_double(state.fps);
      }
    }
    if (!std::isfinite(min_fps)) {
      min_fps = 0.0;
    }

    if (status_msg.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Camera sync OK, delta=%.2f ms, fps[min,max]=[%.2f, %.2f], per_cam_fps={%s}",
        last_sync_delta_ms_, min_fps, max_fps, per_cam_fps.c_str());
    } else if (status_msg.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Camera sync WARN, delta=%.2f ms, fps[min,max]=[%.2f, %.2f], per_cam_fps={%s}",
        last_sync_delta_ms_, min_fps, max_fps, per_cam_fps.c_str());
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Camera sync ERROR, delta=%.2f ms, fps[min,max]=[%.2f, %.2f], per_cam_fps={%s}",
        last_sync_delta_ms_, min_fps, max_fps, per_cam_fps.c_str());
    }
  }

  void add_kv(diagnostic_msgs::msg::DiagnosticStatus & status,
             const std::string & key,
             double value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(value);
    status.values.push_back(kv);
  }

  void set_status(diagnostic_msgs::msg::DiagnosticStatus & status,
                  int level,
                  const std::string & message)
  {
    if (level > status.level) {
      status.level = level;
      status.message = message;
    }
  }

  static std::string short_name(const std::string & full)
  {
    auto pos = full.find_last_of('/');
    if (pos == std::string::npos || pos == 0) {
      return full;
    }
    auto prev = full.find_last_of('/', pos - 1);
    if (prev == std::string::npos) {
      return full.substr(pos + 1);
    }
    return full.substr(prev + 1);
  }

  static std::string format_double(double value)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", value);
    return std::string(buf);
  }

  std::vector<std::string> topic_names_;
  std::string diagnostics_topic_;
  double sync_tolerance_ms_{30.0};
  double min_fps_{15.0};
  int window_size_{30};
  bool use_approx_sync_{true};
  double stale_timeout_sec_{1.0};
  double status_period_sec_{1.0};

  std::vector<TopicState> states_;
  std::vector<rclcpp::Subscription<Image>::SharedPtr> rate_subs_;

  std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> subs_;
  std::unique_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<Image, Image, Image, Image, Image>>> approx_sync_;
  std::unique_ptr<message_filters::Synchronizer<
    message_filters::sync_policies::ExactTime<Image, Image, Image, Image, Image>>> exact_sync_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  bool have_sync_{false};
  rclcpp::Time last_sync_stamp_{0, 0, RCL_STEADY_TIME};
  double last_sync_delta_ms_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraHealthChecker>());
  rclcpp::shutdown();
  return 0;
}
