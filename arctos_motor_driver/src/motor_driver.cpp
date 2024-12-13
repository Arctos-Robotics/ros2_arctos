#include "motor_driver.hpp"
#include <cmath>
#include <map>
#include <stdint.h>
#include <algorithm>

uint16_t calculate_crc(const std::vector<uint8_t>& data)
{
    uint16_t crc = 0xFFFF;  // Initial CRC value
    for (size_t i = 0; i < data.size(); ++i)
    {
        crc ^= data[i] << 8;  // XOR the byte into the CRC (shifted by 8 bits)
        for (int j = 0; j < 8; ++j)  // Process each bit
        {
            if (crc & 0x8000)  // Check the high bit
            {
                crc = (crc << 1) ^ 0x11021;  // Shift and XOR with the polynomial
            }
            else
            {
                crc <<= 1;  // Just shift if no XOR needed
            }
        }
    }
    return crc;
}

std::vector<double> joint_positions_;

MotorDriver::MotorDriver(
)
    : Node("motor_driver")
{
    RCLCPP_INFO(this->get_logger(), "MotorDriver constructor called");
    // Publisher for CAN messages
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_motor_can_bus", 10);

    // Declare motor IDs parameter and initialize
    declare_parameter<std::vector<long int>>("motor_ids", {1});
    get_parameter("motor_ids", motor_ids_);

    // Setup subscriptions and timers for each motor
    for (int64_t motor_id : motor_ids_)  // Use int64_t in the loop
    {
        // Subscription for each motor
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/from_motor_can_bus", 10,
            [this, motor_id](const can_msgs::msg::Frame::SharedPtr msg) {
                this->canMessageCallback(msg, motor_id);
            });

        // Timer to request joint angles periodically for each motor
        auto timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this, motor_id]() { this->requestJointAngle(motor_id); });
        timers_.push_back(timer);
    }
}

void MotorDriver::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg, uint32_t motor_id)
{
        if (msg->id == motor_id && msg->dlc == 8)
    {
        // Combine the two bytes to form the raw angle value
        int32_t raw_angle = (msg->data[0] << 8) | msg->data[1];

        // Convert the raw angle value to radians (angle in float64)
        double angle_radians = (static_cast<double>(raw_angle) / 0x4000) * 2 * M_PI;

        // Resize the joint_positions array if necessary
        if (motor_id >= joint_positions_.size())
        {
            joint_positions_.resize(motor_id + 1, 0.0);  // Resize to accommodate new motor_id if needed
        }

        // Update the joint position for the specific motor
        joint_positions_[motor_id] = angle_radians;
    }
}
void MotorDriver::requestJointAngle(uint32_t motor_id)
{
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 2;
    msg->data = {0x30, 0x31}; // Example request data
    can_pub_->publish(*msg);
}

float MotorDriver::getMotorPosition(uint32_t motor_id)
{
    // Send request for motor position
    motor_position_received_[motor_id] = false;  // Reset the flag before requesting
    requestJointAngle(motor_id);

    // Wait until the position is received
    while (!motor_position_received_[motor_id])
    {
        rclcpp::spin_some(this->get_node_base_interface());  // Process incoming messages
    }

    // Return the motor position once it's updated
    return joint_states_[motor_id].position[0];
}
void MotorDriver::stopMotor(uint32_t motor_id)
{
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = 6;
    msg->data = {0x01, 0xF6, 0x00, 0x00, 0x02, 0xF9}; // Stop command for the motor
    can_pub_->publish(*msg);
}

void MotorDriver::sendPositionCommand(uint32_t motor_id, int32_t abs_position, uint16_t speed, uint8_t acceleration)
{
    // Ensure the parameters are valid
    if (speed < 0 || speed > 3000)
    {
        RCLCPP_ERROR(this->get_logger(), "Speed must be between 0 and 3000 RPM.");
        return;
    }
    if (acceleration > 255)
    {
        RCLCPP_ERROR(this->get_logger(), "Acceleration must be between 0 and 255.");
        return;
    }

    // Pack the absolute position (24-bit) in little-endian format
    uint8_t position_bytes[3];
    position_bytes[0] = (abs_position & 0xFF);
    position_bytes[1] = ((abs_position >> 8) & 0xFF);
    position_bytes[2] = ((abs_position >> 16) & 0xFF);

    // Create the CAN message frame (std::vector)
    std::vector<uint8_t> frame_data = {
        0xFE,                  // Command (move to position)
        static_cast<uint8_t>(speed),   // Speed (RPM)
        acceleration,          // Acceleration
        position_bytes[0],     // Position (byte 1)
        position_bytes[1],     // Position (byte 2)
        position_bytes[2],     // Position (byte 3)
        0x00,                  // Reserved byte (depends on protocol)
    };

    // Calculate CRC and split into two bytes
    uint16_t crc = calculate_crc(frame_data);
    uint8_t crc_low = static_cast<uint8_t>(crc & 0xFF);    // Lower byte of CRC
    uint8_t crc_high = static_cast<uint8_t>((crc >> 8) & 0xFF);  // Upper byte of CRC

    // Add CRC bytes to the frame
    frame_data.push_back(crc_low);
    frame_data.push_back(crc_high);

    // Ensure the frame size is not more than 8 bytes
    if (frame_data.size() > 8)
    {
        RCLCPP_ERROR(this->get_logger(), "Frame data exceeds 8 bytes.");
        return;
    }

    // Create and publish the CAN message
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;  // Motor ID
    msg->dlc = frame_data.size();  // Data length code (DLC)
    
    // Use std::copy to copy data from vector to array
    std::copy(frame_data.begin(), frame_data.end(), msg->data.begin());

    // Publish the message
    can_pub_->publish(*msg);
}


void MotorDriver::setMotorSpeed(uint32_t motor_id, float speed, bool direction, uint8_t acceleration)
{
    // Check that speed is within the allowed range (0-3000 RPM)
    if (speed < 0 || speed > 3000)
    {
        RCLCPP_ERROR(this->get_logger(), "Speed must be between 0 and 3000 RPM.");
        return;
    }

    // Check that acceleration is within the allowed range (0-255)
    if (acceleration > 255)
    {
        RCLCPP_ERROR(this->get_logger(), "Acceleration must be between 0 and 255.");
        return;
    }

    // Convert speed to the appropriate format (combining high and low bits)
    uint8_t high_speed = (static_cast<uint16_t>(speed) >> 4) & 0xFF;  // Upper 8 bits of speed
    uint8_t low_speed = static_cast<uint16_t>(speed) & 0xF;  // Lower 4 bits of speed

    // Direction byte: Set high bit for direction, lower 4 bits as speed
    uint8_t direction_byte = (direction ? 0x80 : 0x00) | high_speed;  // Direction bit and speed high byte
    uint8_t speed_byte = low_speed;  // Speed low byte

    // Acceleration byte (already provided as is)
    uint8_t acc_byte = acceleration;

    // Create CAN message frame data
    std::vector<uint8_t> frame_data = {
        0x01,  // Command byte
        0xF6,  // Speed mode command
        direction_byte,  // Direction + speed high byte
        speed_byte,  // Speed low byte
        acc_byte,  // Acceleration byte
        0x00,  // Reserved byte
    };

    // Calculate CRC and split into two bytes
    uint16_t crc = calculate_crc(frame_data);
    uint8_t crc_low = static_cast<uint8_t>(crc & 0xFF);    // Lower byte of CRC
    uint8_t crc_high = static_cast<uint8_t>((crc >> 8) & 0xFF);  // Upper byte of CRC

    // Add CRC bytes to the frame
    frame_data.push_back(crc_low);
    frame_data.push_back(crc_high);

    // Ensure the frame size is not more than 8 bytes
    if (frame_data.size() > 8)
    {
        RCLCPP_ERROR(this->get_logger(), "Frame data exceeds 8 bytes.");
        return;
    }

    // Create CAN message frame
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;  // Motor ID
    msg->dlc = frame_data.size();  // Data length code (DLC)
    
    // Use std::copy to copy data from vector to array
    std::copy(frame_data.begin(), frame_data.end(), msg->data.begin());

    // Publish the CAN message
    can_pub_->publish(*msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorDriver>();
    //node->sendPositionCommand(1, 0x4000, 600, 2);
    node->setMotorSpeed(1, 0x1F4, 0x80, 2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
