#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>
#include <map>

class MotorDriver : public rclcpp::Node
{
public:
    // Constructor
    MotorDriver();

    // Method to get the motor position
    float getMotorPosition(uint32_t motor_id);

    // Method to request the joint angle from the motor
    void requestJointAngle(uint32_t motor_id);

    // Method to stop the motor
    void stopMotor(uint32_t motor_id);

    // Method to set the motor's position
    void sendPositionCommand(uint32_t motor_id, int32_t abs_position, uint16_t speed, uint8_t acceleration);

    // Method to set the motor's speed and direction
    void setMotorSpeed(uint32_t motor_id, float speed, bool direction, uint8_t acceleration);

private:

    // Publisher for CAN messages
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

    // Subscription for CAN messages
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    // List of motor IDs
    std::vector<long int> motor_ids_;
    std::vector<bool> motor_position_received_;
    // Timer to handle periodic joint angle requests
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

    // A map to hold joint states for each motor
    std::map<uint32_t, sensor_msgs::msg::JointState> joint_states_;

    // Method to process the incoming CAN message
    void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg, uint32_t motor_id);

    // Keep only one declaration of publishJointState
    void publishJointState(uint32_t motor_id);
};

#endif // MOTOR_DRIVER_HPP
