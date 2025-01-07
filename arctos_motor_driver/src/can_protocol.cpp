#include "arctos_motor_driver/can_protocol.hpp"
#include "arctos_motor_driver/motor_types.hpp"
#include <cmath>

namespace arctos_motor_driver {

CANProtocol::CANProtocol(rclcpp::Node::SharedPtr node) {
    can_pub_ = node->create_publisher<can_msgs::msg::Frame>("/to_motor_can_bus", 10);
}

void CANProtocol::sendFrame(uint8_t motor_id, const std::vector<uint8_t>& data) {
    auto msg = std::make_shared<can_msgs::msg::Frame>();
    msg->id = motor_id;
    msg->dlc = static_cast<uint8_t>(data.size() + 1);  // +1 for CRC byte
    
    // Initialize array with zeros
    std::fill(msg->data.begin(), msg->data.end(), 0);
    
    // Copy command data
    std::copy(data.begin(), data.end(), msg->data.begin());
    
    // Calculate CRC: ID + all data bytes
    uint16_t crc = motor_id;  // Start with motor ID
    for (size_t i = 0; i < data.size(); i++) {
        crc += data[i];
    }
    crc &= 0xFF;  // Keep only lower byte
    
    // Add CRC as last byte
    msg->data[data.size()] = static_cast<uint8_t>(crc);
    
    can_pub_->publish(*msg);
}

// uint16_t CANProtocol::calculateCRC(const uint8_t* data, size_t length) {
//     uint16_t crc = 0x00;
//     for (size_t i = 0; i < length; ++i) {
//         crc += data[i];
//     }
//     return static_cast<uint16_t>(crc & 0xFF);
// }

// https://discord.com/channels/1099629962618748958/1126799425205973012/1239584322122416179
// double CANProtocol::decodeInt48(const std::vector<uint8_t>& data) {
//     if (data.size() != 6) {
//         throw std::runtime_error("Data size must be 6 bytes for int48_t decoding");
//     }
    
//     std::cout << "Input bytes: ";
//     for (int i = 0; i < 6; ++i) {
//         std::cout << std::hex << static_cast<int>(data[i]) << " ";
//     }
//     std::cout << std::dec << std::endl;
    
//     // First three bytes represent the carry value
//     int32_t carry = (data[0] << 16) | (data[1] << 8) | data[2];
//     std::cout << "Carry value: " << carry << std::endl;
    
//     // Second three bytes represent the encoder value
//     uint32_t encoder = (data[3] << 16) | (data[4] << 8) | data[5];
//     std::cout << "Encoder value: 0x" << std::hex << encoder << std::dec << std::endl;

//     // Full rotations from carry
//     double fullRevolutions = static_cast<double>(carry) * 360.0;
//     std::cout << "Degrees from carry: " << fullRevolutions << std::endl;

//     // Partial rotation from encoder
//     double partialRevolution = (static_cast<double>(encoder) * 360.0) / 0x4000;
//     std::cout << "Degrees from encoder: " << partialRevolution << std::endl;
    
//     double total = fullRevolutions + partialRevolution;
//     std::cout << "Total degrees: " << total << std::endl;
    
//     return total;
// }

double CANProtocol::decodeInt48(const std::vector<uint8_t>& data) {
    if (data.size() != 6) {
        throw std::runtime_error("Data size must be 6 bytes for int48_t decoding");
    }

    // Decode carry (signed 24-bit integer)
    int32_t carry = (data[0] << 16) | (data[1] << 8) | data[2];
    if (carry & 0x800000) {  // Sign-extend if negative
        carry |= 0xFF000000;
    }

    // Decode encoder value (unsigned 24-bit integer)
    uint32_t encoder = (data[3] << 16) | (data[4] << 8) | data[5];
    encoder %= 0x4000;  // Wrap within 16,384 steps

    // Compute absolute position in steps
    int64_t absolute_position_steps = (static_cast<int64_t>(carry) * 0x4000) + encoder;

    // Convert to degrees
    double degrees = (static_cast<double>(absolute_position_steps) * 360.0) / 0x4000;

    return degrees;
}


double CANProtocol::decodeVelocityToRPM(const std::vector<uint8_t>& data) {
    if (data.size() != 2) {
        throw std::runtime_error("Invalid data size for velocity decoding. Expected 2 bytes.");
    }
    
    // RPM data is sent as a 16-bit signed integer
    int16_t speed = static_cast<int16_t>((data[1] << 8) | data[0]);
    return static_cast<double>(speed);
}

double CANProtocol::encoderToRadians(double encoder_value) {
    // Convert encoder degrees to radians
    return encoder_value * MotorConstants::DEG_TO_RAD;
}

double CANProtocol::radiansToEncoder(double radians) {
    // Convert radians to encoder degrees
    return radians * MotorConstants::RAD_TO_DEG;
}

double CANProtocol::rpmToRadPS(double rpm) {
    // Convert RPM to radians per second
    return rpm * MotorConstants::RPM_TO_RADPS;
}

double CANProtocol::radPSToRPM(double rad_ps) {
    // Convert radians per second to RPM
    return rad_ps * MotorConstants::RADPS_TO_RPM;
}

} // namespace arctos_motor_driver