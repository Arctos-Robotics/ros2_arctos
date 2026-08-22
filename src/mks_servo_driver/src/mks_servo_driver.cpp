#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

#include <array>
#include <cstddef>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>

namespace {
constexpr uint8_t CMD_SET_POSITION = 0x10;
constexpr uint8_t CMD_SET_VELOCITY = 0x11;
constexpr uint8_t CMD_STOP = 0x12;
constexpr uint8_t CMD_READ_POSITION = 0x14;
constexpr uint8_t CMD_READ_VELOCITY = 0x15;
constexpr uint8_t RES_POSITION = 0x90;
constexpr uint8_t RES_VELOCITY = 0x91;
constexpr uint8_t RES_POSITION_ACK = 0x92;
constexpr uint8_t RES_VELOCITY_ACK = 0x93;

std::array<uint8_t, 8> make_frame(uint8_t command, int32_t value) {
  std::array<uint8_t, 8> data{};
  data[0] = command;
  data[1] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(value & 0xFF);
  return data;
}

int32_t read_int32(const std::array<uint8_t, 8> & data, std::size_t offset) {
  if (offset + 4u > data.size()) {
    return 0;
  }
  return static_cast<int32_t>(static_cast<uint32_t>(data[offset]) << 24 |
                             static_cast<uint32_t>(data[offset + 1]) << 16 |
                             static_cast<uint32_t>(data[offset + 2]) << 8 |
                             static_cast<uint32_t>(data[offset + 3]));
}
} // namespace

class MksServoDriver {
public:
  explicit MksServoDriver(rclcpp::Node & node)
      : node_(node),
        motor_id_(1),
        angle_scale_(1.0),
        velocity_scale_(1.0),
        request_timeout_ms_(100)
  {
    node_.declare_parameter<int>("motor_id", 1);
    node_.declare_parameter<double>("angle_scale", 1.0);
    node_.declare_parameter<double>("velocity_scale", 1.0);
    node_.declare_parameter<int>("request_timeout_ms", 100);
    node_.get_parameter("motor_id", motor_id_);
    node_.get_parameter("angle_scale", angle_scale_);
    node_.get_parameter("velocity_scale", velocity_scale_);
    node_.get_parameter("request_timeout_ms", request_timeout_ms_);
    publisher_ = node_.create_publisher<can_msgs::msg::Frame>("/To_motor_can_bus", rclcpp::QoS(10));
    subscription_ = node_.create_subscription<can_msgs::msg::Frame>(
        "/From_motor_can_bus", rclcpp::QoS(10), std::bind(&MksServoDriver::on_frame, this, std::placeholders::_1));
    timer_ = node_.create_wall_timer(std::chrono::milliseconds(1), std::bind(&MksServoDriver::on_timer, this));
  }

  void set_desired_angle(double angle) {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_angle_ = angle;
    last_position_command_ = now();
    position_ack_ = false;
    send_frame_locked(CMD_SET_POSITION, to_counts(angle, angle_scale_));
  }

  void set_desired_velocity(double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_velocity_ = velocity;
    last_velocity_command_ = now();
    velocity_ack_ = false;
    send_frame_locked(CMD_SET_VELOCITY, to_counts(velocity, velocity_scale_));
  }

  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    send_frame_locked(CMD_STOP, 0);
  }

  double current_angle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_angle_;
  }

  double current_velocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_velocity_;
  }

  double desired_angle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return desired_angle_;
  }

  double desired_velocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return desired_velocity_;
  }

  bool position_confirmed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_ack_ && fresh(last_position_ack_);
  }

  bool velocity_confirmed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return velocity_ack_ && fresh(last_velocity_ack_);
  }

  bool position_online() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fresh(last_position_response_);
  }

  bool velocity_online() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return fresh(last_velocity_response_);
  }

private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  static TimePoint now() {
    return Clock::now();
  }

  bool fresh(TimePoint point) const {
    if (point == TimePoint()) {
      return false;
    }
    return now() - point < std::chrono::milliseconds(request_timeout_ms_);
  }

  static int32_t to_counts(double value, double scale) {
    if (scale <= 0.0) {
      return 0;
    }
    return static_cast<int32_t>(value / scale);
  }

  static double from_counts(int32_t value, double scale) {
    if (scale <= 0.0) {
      return 0.0;
    }
    return static_cast<double>(value) * scale;
  }

  void on_timer() {
    std::lock_guard<std::mutex> lock(mutex_);
    send_frame_locked(CMD_READ_POSITION, 0);
    send_frame_locked(CMD_READ_VELOCITY, 0);
    if (last_position_command_ != TimePoint() &&
        now() - last_position_command_ > std::chrono::milliseconds(request_timeout_ms_)) {
      position_ack_ = false;
    }
    if (last_velocity_command_ != TimePoint() &&
        now() - last_velocity_command_ > std::chrono::milliseconds(request_timeout_ms_)) {
      velocity_ack_ = false;
    }
  }

  void on_frame(const can_msgs::msg::Frame::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->id != static_cast<uint32_t>(motor_id_)) {
      return;
    }
    if (msg->dlc < 1u) {
      return;
    }
    const uint8_t command = msg->data[0];
    if (command == RES_POSITION) {
      current_angle_ = from_counts(read_int32(msg->data, 1u), angle_scale_);
      last_position_response_ = now();
    } else if (command == RES_VELOCITY) {
      current_velocity_ = from_counts(read_int32(msg->data, 1u), velocity_scale_);
      last_velocity_response_ = now();
    } else if (command == RES_POSITION_ACK) {
      position_ack_ = true;
      last_position_ack_ = now();
    } else if (command == RES_VELOCITY_ACK) {
      velocity_ack_ = true;
      last_velocity_ack_ = now();
    }
  }

  bool send_frame_locked(uint8_t command, int32_t value) {
    can_msgs::msg::Frame frame;
    frame.id = static_cast<uint32_t>(motor_id_);
    frame.dlc = 8;
    frame.data = make_frame(command, value);
    return publisher_->publish(frame);
  }

  rclcpp::Node & node_;
  int motor_id_;
  double angle_scale_;
  double velocity_scale_;
  int request_timeout_ms_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  double current_angle_{0.0};
  double current_velocity_{0.0};
  double desired_angle_{0.0};
  double desired_velocity_{0.0};
  bool position_ack_{false};
  bool velocity_ack_{false};
  TimePoint last_position_command_{};
  TimePoint last_velocity_command_{};
  TimePoint last_position_response_{};
  TimePoint last_velocity_response_{};
  TimePoint last_position_ack_{};
  TimePoint last_velocity_ack_{};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mks_servo_driver");
  auto driver = std::make_shared<MksServoDriver>(*node);
  (void)driver;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
