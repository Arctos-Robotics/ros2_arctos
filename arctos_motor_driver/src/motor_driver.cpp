#include "arctos_motor_driver/motor_driver.hpp"
#include <cmath>
#include <map>
#include <stdint.h>
#include <algorithm>
#include "rclcpp/executors.hpp"

namespace arctos_motor_driver
{
    MotorDriver::MotorDriver()
        : Node("motor_driver"), motor_id(0)
    {
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_motor_can_bus", 10);
        RCLCPP_INFO(this->get_logger(), "CAN publisher initialized.");

        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/from_motor_can_bus", 10,
            [this](const can_msgs::msg::Frame::SharedPtr msg) {
                this->canMessageCallback(msg);
            });
        RCLCPP_INFO(this->get_logger(), "CAN callback initialized.");
        auto timer = this->create_wall_timer(
                std::chrono::milliseconds(10),  // 0.01 second interval
                [this]() {
                    // Non-blocking check for new messages
                    rclcpp::spin_some(this->get_node_base_interface());

                    // Request angle if no new message has arrived
                    if (!message_received) {
                        this->requestAngle();
                    }
                    // Continue other code here
                    // Any other logic can go here without waiting for the callback
                });

    }

    int64_t MotorDriver::decodeInt48(const std::vector<uint8_t>& data)
    {
        if (data.size() != 6) {
            throw std::runtime_error("Data size must be 6 bytes for int48_t decoding");
        }

        // Combine 6 bytes into a 48-bit signed integer
        int64_t value = 0;
        for (size_t i = 0; i < 6; ++i) {
            value = (value << 8) | data[i];
        }

        // Check if the highest bit (sign bit) is set (negative value)
        if (value & (1LL << 47)) {
            value |= ~((1LL << 48) - 1); // Sign extend for negative values
        }

        return value;
    }

    void MotorDriver::requestAngle()
    {
        auto msg = std::make_shared<can_msgs::msg::Frame>();
        msg->id = motor_id;
        msg->dlc = 2;
        msg->data = {0x31, 0x32, 0, 0, 0, 0, 0, 0};
        RCLCPP_INFO(this->get_logger(), "Publishing CAN message with ID");
        
        // Ensure the timer is non-blocking
        rclcpp::spin_some(this->get_node_base_interface());  // Process any pending messages

        can_pub_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published!");
    }

    void MotorDriver::requestVelocity()
    {
        auto msg = std::make_shared<can_msgs::msg::Frame>();
        msg->id = motor_id;
        msg->dlc = 2;
        msg->data = {0x32, 0x33};
        can_pub_->publish(*msg);
    }

    // void MotorDriver::setAngle()
    // {
    // // Ensure the parameters are valid
    //     if (speed < 0 || speed > 3000)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Speed must be between 0 and 3000 RPM.");
    //         return;
    //     }
    //     if (acceleration > 255)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Acceleration must be between 0 and 255.");
    //         return;
    //     }

    //     // Pack the absolute position (24-bit) in little-endian format
    //     uint8_t position_bytes[3];
    //     position_bytes[0] = (abs_position & 0xFF);
    //     position_bytes[1] = ((abs_position >> 8) & 0xFF);
    //     position_bytes[2] = ((abs_position >> 16) & 0xFF);

    //     // Create the CAN message frame (std::vector)
    //     std::vector<uint8_t> frame_data = {
    //         0xFE,                  // Command (move to position)
    //         static_cast<uint8_t>(speed),   // Speed (RPM)
    //         acceleration,          // Acceleration
    //         position_bytes[0],     // Position (byte 1)
    //         position_bytes[1],     // Position (byte 2)
    //         position_bytes[2],     // Position (byte 3)
    //         0x00,                  // Reserved byte (depends on protocol)
    //     };

    //     // Calculate CRC and split into two bytes
    //     uint16_t crc = calculate_crc(frame_data);
    //     uint8_t crc_low = static_cast<uint8_t>(crc & 0xFF);    // Lower byte of CRC
    //     uint8_t crc_high = static_cast<uint8_t>((crc >> 8) & 0xFF);  // Upper byte of CRC

    //     // Add CRC bytes to the frame
    //     frame_data.push_back(crc_low);
    //     frame_data.push_back(crc_high);

    //     // Create and publish the CAN message
    //     auto msg = std::make_shared<can_msgs::msg::Frame>();
    //     msg->id = motor_id;  // Motor ID
    //     msg->dlc = frame_data.size();  // Data length code (DLC)
        
    //     // Use std::copy to copy data from vector to array
    //     std::copy(frame_data.begin(), frame_data.end(), msg->data.begin());

    //     // Publish the message
    //     can_pub_->publish(*msg);

    // }

    // void MotorDriver::setVelocity()
    // {
    //     // Check that speed is within the allowed range (0-3000 RPM)
    //     if (speed < 0 || speed > 3000)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Speed must be between 0 and 3000 RPM.");
    //         return;
    //     }

    //     // Check that acceleration is within the allowed range (0-255)
    //     if (acceleration > 255)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Acceleration must be between 0 and 255.");
    //         return;
    //     }

    //     // Convert speed to the appropriate format (combining high and low bits)
    //     uint8_t high_speed = (static_cast<uint16_t>(speed) >> 4) & 0xFF;  // Upper 8 bits of speed
    //     uint8_t low_speed = static_cast<uint16_t>(speed) & 0xF;  // Lower 4 bits of speed

    //     // Direction byte: Set high bit for direction, lower 4 bits as speed
    //     uint8_t direction_byte = (direction ? 0x80 : 0x00) | high_speed;  // Direction bit and speed high byte
    //     uint8_t speed_byte = low_speed;  // Speed low byte

    //     // Acceleration byte (already provided as is)
    //     uint8_t acc_byte = acceleration;
    //     position_sub_ = this->create_subscription<std_msgs::msg::String>(
    //         "/motor_position_topic", 10, std::bind(&MotorDriver::MotorPositionCallback, this, std::placeholders::_1));


    //     // Create CAN message frame data
    //     std::vector<uint8_t> frame_data = {
    //         0x01,  // Command byte
    //         0xF6,  // Speed mode command
    //         direction_byte,  // Direction + speed high byte
    //         speed_byte,  // Speed low byte
    //         acc_byte,  // Acceleration byte
    //         0x00,  // Reserved byte
    //     };

    //     // Calculate CRC and split into two bytes
    //     uint16_t crc = calculate_crc(frame_data);
    //     uint8_t crc_low = static_cast<uint8_t>(crc & 0xFF);    // Lower byte of CRC
    //     uint8_t crc_high = static_cast<uint8_t>((crc >> 8) & 0xFF);  // Upper byte of CRC

    //     // Add CRC bytes to the frame
    //     frame_data.push_back(crc_low);
    //     frame_data.push_back(crc_high);

    //     // Ensure the frame size is not more than 8 bytes
    //     if (frame_data.size() > 8)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Frame data exceeds 8 bytes.");
    //         return;
    //     }

    //     // Create CAN message frame
    //     auto msg = std::make_shared<can_msgs::msg::Frame>();
    //     msg->id = motor_id;  // Motor ID
    //     msg->dlc = frame_data.size();  // Data length code (DLC)
        
    //     // Use std::copy to copy data from vector to array
    //     std::copy(frame_data.begin(), frame_data.end(), msg->data.begin());

    //     // Publish the CAN message
    //     can_pub_->publish(*msg);
    // }

    void MotorDriver::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        const std::vector<uint8_t> can_data(msg->data.begin(), msg->data.end());

        if (can_data.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Received incomplete CAN message");
            return;
        }

        // Check for identifiers that indicate position or velocity data
        uint8_t identifier = can_data[0];

        if (identifier == 0x31) {
            // Process position data (e.g., encoder value)
            RCLCPP_INFO(this->get_logger(), "Processing encoder value...");

            // Decode the 48-bit value from bytes 1-6
            angle = decodeInt48(can_data);

            // Log the encoder value
            RCLCPP_INFO(this->get_logger(), "Decoded encoder value: %ld", angle);

            // Additional processing (e.g., update position, calculate angle) can go here
        } else if (identifier == 0x32 || identifier == 0x33) {


        } else {
        }
    }

    // float MotorDriver::MotorPositionCallback(const std_msgs::msg::String::SharedPtr msg)
    // {
    //     const std::vector<uint8_t> can_data = {msg->data.begin(), msg->data.end()};

    //     if (can_data.size() < 8) {
    //         RCLCPP_WARN(this->get_logger(), "Received incomplete CAN message");
    //         return;
    //     }

    //     // Check for identifiers 0x48 or 0x31
    //     uint8_t identifier = can_data[0];
    //     if (identifier == 0x48 || identifier == 0x31) {
    //         RCLCPP_INFO(this->get_logger(), "Processing encoder value...");

    //         // Decode the 48-bit value from bytes 1-6
    //         int64_t encoder_value = decodeInt48(can_data);

    //         // Log the encoder value
    //         RCLCPP_INFO(this->get_logger(), "Decoded encoder value: %ld", encoder_value);

    //         // Additional processing can be done here
    //     } else {
    //         RCLCPP_DEBUG(this->get_logger(), "Ignoring message with identifier: 0x%02X", identifier);
    //     }
    // }

    // float MotorDriver::MotorVelocityCallback(const std_msgs::msg::String::SharedPtr msg)    
    // {


    // }

    void MotorDriver::stopMotor()
    {
        auto msg = std::make_shared<can_msgs::msg::Frame>();
        msg->id = motor_id;
        msg->dlc = 6;
        msg->data = {0x01, 0xF6, 0x00, 0x00, 0x02, 0xF9};
        can_pub_->publish(*msg);
    }


    uint16_t calculate_crc(const std::vector<uint8_t>& data)
    {
        uint16_t crc = 0xFFFF;  // Initial CRC value
        for (size_t i = 0; i < data.size(); ++i)
        {
            crc ^= data[i] << 8;  
            for (int j = 0; j < 8; ++j)  
            {
                if (crc & 0x8000)  
                {
                    crc = (crc << 1) ^ 0x11021;  
                }
                else
                {
                    crc <<= 1;  
                }
            }
        }
        return crc;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arctos_motor_driver::MotorDriver>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting rclcpp::spin...");
    exec.spin();
    RCLCPP_INFO(node->get_logger(), "Exiting rclcpp::spin...");

    rclcpp::shutdown();
    return 0;
}
