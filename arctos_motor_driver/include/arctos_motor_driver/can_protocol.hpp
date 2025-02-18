#ifndef ARCTOS_CAN_PROTOCOL_HPP_
#define ARCTOS_CAN_PROTOCOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>
#include <cstdint>

namespace arctos_motor_driver {

class CANProtocol {
public:
    explicit CANProtocol(rclcpp::Node::SharedPtr node);
    virtual ~CANProtocol() = default;
    
    // Make sendFrame virtual and pure
    virtual void sendFrame(uint8_t motor_id, const std::vector<uint8_t>& data);
    
    // Other methods remain the same
    // uint16_t calculateCRC(const uint8_t* data, size_t length);
    
    // Data decoding methods - made static for easier testing
    static double decodeInt48(const std::vector<uint8_t>& data);
    static double decodeVelocityToRPM(const std::vector<uint8_t>& data);
    
    // Unit conversions - made static for easier testing
    static double encoderToRadians(double encoder_value);
    static double radiansToEncoder(double radians);
    static double rpmToRadPS(double rpm);
    static double radPSToRPM(double rad_ps);

protected:
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_CAN_PROTOCOL_HPP_