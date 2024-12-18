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

        MotorDriver();    

        void requestAngle();
        void requestVelocity();
        // void setAngle();
        void setVelocity(uint8_t direction, uint16_t speed, uint8_t acceleration);
        void stopMotor();

        // Callbacks for motor position and velocity
        void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);

        int motor_id;
        double angle;
        double velocity;
        uint16_t speed;
        uint8_t acceleration;
        bool direction;
        uint32_t abs_position;

        void setMotorId(int id) {
            motor_id = id;
        }    
        uint16_t calculate_crc(const uint8_t* data, size_t length);
    private:
        // Publisher for CAN messages
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
        
        // Subscription to CAN messages
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

        // Timer for periodic requests
        rclcpp::TimerBase::SharedPtr request_timer_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr velocity_timer_;
        // Helper methods
        double decodeVelocityToRPM(const std::vector<uint8_t>& data);        
        double decodeInt48(const std::vector<uint8_t>& data);  // Decode 48-bit integer from CAN message
    };
}

#endif  // MOTOR_DRIVER_HPP
