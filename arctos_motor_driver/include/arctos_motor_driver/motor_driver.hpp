#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

namespace arctos_motor_driver  // Namespace for the motor driver
{
    class MotorDriver : public rclcpp::Node
    {
    public:
        // Constructor
        MotorDriver();    
        // explicit MotorDriver(int motor_id);

        // Methods to interact with the motor
        void requestAngle();  // Request motor angle
        void requestVelocity();  // Request motor velocity
        // void setAngle();  // Set motor angle
        // void setVelocity();  // Set motor velocity
        void stopMotor();  // Stop motor

        // Callbacks for motor position and velocity
        void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
        int motor_id;  // Motor ID
        int64_t angle;  // Motor angle
        int64_t velocity;  // Motor velocity
        uint16_t speed;  // Speed setting for motor
        uint8_t acceleration;  // Acceleration setting for motor
        bool direction;  // Direction of motor rotation
        uint32_t abs_position;

    private:

        // Publisher for CAN messages
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
        
        // Subscription to CAN messages
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        // Timer for periodic requests
        rclcpp::TimerBase::SharedPtr request_timer_;

        // Helper methods
        int64_t decodeInt48(const std::vector<uint8_t>& data);  // Decode 48-bit integer from CAN message
        uint16_t calculate_crc(const std::vector<uint8_t>& data);  // Calculate CRC for CAN frame
    };
}

#endif  // MOTOR_DRIVER_HPP
