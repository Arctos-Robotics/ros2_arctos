#ifndef ARCTOS_CAN_PROTOCOL_HPP_
#define ARCTOS_CAN_PROTOCOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <vector>
#include <cstdint>
#include <queue>

namespace arctos_motor_driver {

class CANProtocol {
public:
    explicit CANProtocol(rclcpp::Node::SharedPtr node);
    virtual ~CANProtocol() = default;
    
    /**
     * @brief Processes an encoder response from a motor.  Make sendFrame virtual and pure
     * @param motor_id The ID of the motor.
     * @param data The data will be send on the bus
     */
    virtual void sendFrame(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief callback function to store received CAN to internal queue
     * @param msg The received frame
     */
    void canCallback(const can_msgs::msg::Frame::SharedPtr msg);

    /**
     * @brief Check if there's any frame in internal queue
     * @return true if queue is not empty
     */
    bool frameInQueue();

    /**
     * @brief Get the frame in the internal queue, store them in data
     * @param data The frame got from internal queue.
     * @return true if get frame from internal queue successfully.
     */
    bool getFrame(can_msgs::msg::Frame::SharedPtr& data);
    
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
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

private:
    rclcpp::Node::SharedPtr node_; /**< A shared pointer to the ROS 2 node. */
    std::queue<can_msgs::msg::Frame::SharedPtr> can_message_queue_;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_CAN_PROTOCOL_HPP_