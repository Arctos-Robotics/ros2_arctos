#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include <vector>

class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver();

private:
    int motor_id;
    float angle;
    float velocity;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::TimerBase::SharedPtr request_timer_;

    void requestAngle();
    void stopMotor();
    void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
    int64_t decodeInt48(const std::vector<uint8_t> &data);
    uint16_t calculate_crc(const std::vector<uint8_t> &data);
};

#endif
