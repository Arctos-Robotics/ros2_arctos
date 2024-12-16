#include "arctos_motor_driver/motor_driver.hpp"
#include <cmath>
#include <map>
#include <stdint.h>
#include <algorithm>
#include "rclcpp/executors.hpp"

namespace arctos_motor_driver {

MotorDriver::MotorDriver()
    : Node("motor_driver"){
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_motor_can_bus", 10);

    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_motor_can_bus", 10,
        [this](const can_msgs::msg::Frame::SharedPtr msg) {
            this->canMessageCallback(msg);
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotorDriver::requestAngle, this));

    velocity_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotorDriver::requestVelocity, this));
}

double MotorDriver::decodeInt48(const std::vector<uint8_t>& data) {
    if (data.size() != 6) {
        throw std::runtime_error("Data size must be 6 bytes for int48_t decoding");
    }
    int64_t value = 0;
    for (size_t i = 0; i < 6; ++i) {
        value = (value << 8) | data[i];
    }
    if (value & (1LL << 47)) {
        value |= ~((1LL << 48) - 1);
    }
    const double scale_factor = 360.0 / 0x4000;
    double angle_in_degrees = value * scale_factor;

    angle_in_degrees = fmod(angle_in_degrees, 360.0);
    if (angle_in_degrees < 0) {
        angle_in_degrees += 360.0;
    }
    return angle_in_degrees;
}

double MotorDriver::decodeVelocityToRPM(const std::vector<uint8_t>& data) {

    if (data.size() == 2) {
        uint8_t velocity_byte1 = data[0];
        uint8_t velocity_byte2 = data[1];
        int16_t speed = static_cast<int16_t>((velocity_byte2 << 8) | velocity_byte1);
        return static_cast<double>(speed);
    } else {
        throw std::runtime_error("Invalid data size for velocity decoding. Expected 2 bytes.");
    }
}

void MotorDriver::requestAngle() {
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 2;
    msg->data = {0x31, 0x32, 0, 0, 0, 0, 0, 0};
    can_pub_->publish(*msg);
}

void MotorDriver::requestVelocity() {
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 2;
    msg->data = {0x32, 0x33};
    can_pub_->publish(*msg);
}

void MotorDriver::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    const std::vector<uint8_t> can_data(msg->data.begin(), msg->data.end());

    if (can_data.size() < 2) {
        return;
    }
    uint8_t identifier = can_data[0];

    if (identifier == 0x32) {
        if (can_data.size() == 8) {  
            std::vector<uint8_t> velocity_data(can_data.begin() + 1, can_data.begin() + 3);
            try {
                double rpm = decodeVelocityToRPM(velocity_data);
                //RCLCPP_INFO(this->get_logger(), "Decoded Velocity (RPM): %f", rpm);
            } catch (const std::runtime_error& e) {
                RCLCPP_WARN(this->get_logger(), "Error decoding velocity: %s", e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected frame size for velocity data (expected 8 bytes)");
        }
    }

    else if (identifier == 0x31) {
        if (can_data.size() == 8) {
            std::vector<uint8_t> angle_data(can_data.begin() + 1, can_data.begin() + 7);

            try {
                angle = decodeInt48(angle_data);  // Decode the 48-bit angle
                RCLCPP_INFO(this->get_logger(), "Decoded angle value: %f", angle);
            } catch (const std::runtime_error& e) {
                RCLCPP_WARN(this->get_logger(), "Error decoding angle: %s", e.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unexpected frame size for angle data (expected 8 bytes)");
        }
    }
}


void MotorDriver::setVelocity(uint8_t direction, uint16_t speed, uint8_t acceleration) {

    if (speed < 0 || speed > 3000) {
        RCLCPP_WARN(this->get_logger(), "Invalid speed. Must be between 0 and 3000.");
        return;
    }
    if (acceleration < 0 || acceleration > 255) {
        RCLCPP_WARN(this->get_logger(), "Invalid acceleration. Must be between 0 and 255.");
        return;
    }
    uint8_t dir_byte = 0x00;
    if (direction == 1) {
        dir_byte |= 0x00;
    } else if (direction == 2) {
        dir_byte |= 0x80;
    }

    uint8_t speed_high_nibble = (speed >> 8) & 0x0F;
    dir_byte |= speed_high_nibble;
    uint8_t speed_low_byte = speed & 0xFF;
    uint8_t msg_data[6] = {motor_id, 0xF6, dir_byte, speed_low_byte, acceleration, 0x00};
    uint16_t crc = calculate_crc(msg_data, 5);
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 5;
    msg->data = {msg_data[1], msg_data[2], msg_data[3], msg_data[4], static_cast<uint8_t>(crc & 0xFF)};

    std::stringstream frame_data;
    for (const auto& byte : msg->data) {
        frame_data << "0x" << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << " ";
    }
    can_pub_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Sent speed mode command: Speed = %d, Acceleration = %d, Direction = %d", speed, acceleration, direction);
}

void MotorDriver::stopMotor() {
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 5;
    msg->data = {0xF6, 0x00, 0x00, 0x02, 0xF9};
    can_pub_->publish(*msg);
}

uint16_t MotorDriver::calculate_crc(const uint8_t* data, size_t length) {
    uint16_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc += data[i];
    }
    crc &= 0xFF;
    RCLCPP_INFO(this->get_logger(), "CRC = 0x%02X", crc);
    return static_cast<uint16_t>(crc);
}

} // namespace arctos_motor_driver

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arctos_motor_driver::MotorDriver>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    // Spin the node
    exec.spin();

    // Stop the motor before shutting down
    RCLCPP_INFO(node->get_logger(), "Stopping motor before shutdown.");
    node->stopMotor();

    // Ensure the stopMotor command is processed by the CAN bus
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time) < std::chrono::milliseconds(500)) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    rclcpp::shutdown();
    return 0;
}