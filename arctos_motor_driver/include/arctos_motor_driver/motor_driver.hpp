#ifndef ARCTOS_MOTOR_DRIVER_HPP_
#define ARCTOS_MOTOR_DRIVER_HPP_

#include "arctos_motor_driver/motor_types.hpp"
#include "arctos_motor_driver/can_protocol.hpp"
#include <map>
#include <memory>

/**
 * @file motor_driver.hpp
 * @brief This file contains the declaration of the MotorDriver class.
 */

namespace arctos_motor_driver {

/**
 * @class MotorDriver
 * @brief The MotorDriver class provides an interface for controlling motors.
 */
class MotorDriver {
public:
    /**
     * @brief Constructs a MotorDriver object.
     * @param node A shared pointer to the ROS 2 node.
     */
    explicit MotorDriver(rclcpp::Node::SharedPtr node);

    /**
     * @brief Destroys the MotorDriver object.
     */
    ~MotorDriver();

    void processCANMessage(const can_msgs::msg::Frame::SharedPtr msg);
    // Joint management

    void setCAN(std::shared_ptr<CANProtocol> can_protocol);
    
    /**
     * @brief Updates the states of the joints.
     */
    void updateJointStates();

    /**
     * @brief Adds a joint to the motor driver.
     * @param joint_name The name of the joint.
     * @param motor_id The ID of the motor.
     */
    void addJoint(const std::string& joint_name, uint8_t motor_id, std::string hardware_type, double gear_ratio = 1.0);

    /**
     * @brief Removes a joint from the motor driver.
     * @param joint_name The name of the joint to remove.
     */
    void removeJoint(const std::string& joint_name);
    
    // Core control functions

    /**
     * @brief Sets the position of a joint.
     * @param joint_name The name of the joint.
     * @param position The desired position.
     */
    void setJointPosition(const std::string& joint_name, double position, double acceleration = -1.0);

    /**
     * @brief Sets the velocity of a joint.
     * @param joint_name The name of the joint.
     * @param velocity The desired velocity.
     */
    void setJointVelocity(const std::string& joint_name, double velocity);

    /**
     * @brief Gets the position of a joint.
     * @param joint_name The name of the joint.
     * @return The position of the joint.
     */
    double getJointPosition(const std::string& joint_name) const;

    /**
     * @brief Gets the velocity of a joint.
     * @param joint_name The name of the joint.
     * @return The velocity of the joint.
     */
    double getJointVelocity(const std::string& joint_name) const;
    
    // Motor control
    
    /**
     * @brief Enable shaft protection for a joint.
     * @param joint_name The name of the joint.
     */
    void enableShaftProtection(const std::string& joint_name);
    
    /**
     * @brief Enables a motor.
     * @param joint_name The name of the joint associated with the motor.
     */
    void enableMotor(const std::string& joint_name);

    /**
     * @brief Disables a motor.
     * @param joint_name The name of the joint associated with the motor.
     */
    void disableMotor(const std::string& joint_name);

    /**
     * @brief Stops a motor.
     * @param joint_name The name of the joint associated with the motor.
     */
    void stopMotor(const std::string& joint_name);

    /**
     * @brief Stops all motors.
     */
    void stopAllMotors();

    // Parameter management

    /**
     * @brief Gets the status of a motor.
     * @param joint_name The name of the joint associated with the motor.
     * @return The status of the motor.
     */
    MotorStatus getMotorStatus(const std::string& joint_name) const;

    /**
     * @brief Gets the parameters of a motor.
     * @param joint_name The name of the joint associated with the motor.
     * @return The parameters of the motor.
     */
    MotorParameters getMotorParameters(const std::string& joint_name) const;

    /**
     * @brief Sets the working mode of a motor.
     * @param joint_name The name of the joint associated with the motor.
     * @param mode The desired working mode.
     */
    void setWorkingMode(const std::string& joint_name, MotorMode mode);

    /**
     * @brief Sets the working current of a motor.
     * @param joint_name The name of the joint associated with the motor.
     * @param current_ma The desired working current in milliamperes.
     */
    void setWorkingCurrent(const std::string& joint_name, uint16_t current_ma);

    /**
     * @brief Sets the holding current of a motor.
     * @param joint_name The name of the joint associated with the motor.
     * @param percentage The desired holding current as a percentage of the working current.
     */
    void setHoldingCurrent(const std::string& joint_name, uint8_t percentage);

    /**
     * @brief Sets the limits of a joint.
     * @param joint_name The name of the joint.
     * @param pos_min The minimum position.
     * @param pos_max The maximum position.
     * @param vel_max The maximum velocity.
     * @param acc_max The maximum acceleration.
     */
    void setJointLimits(const std::string& joint_name, 
                       double pos_min, double pos_max,
                       double vel_max, double acc_max);

    // Calibration and homing

    /**
     * @brief Calibrates a motor.
     * @param joint_name The name of the joint associated with the motor.
     */
    void calibrateMotor(const std::string& joint_name);

    /**
     * @brief Homes a motor.
     * @param joint_name The name of the joint associated with the motor.
     */
    void homeMotor(const std::string& joint_name);

    /**
     * @brief Checks if a motor is ready.
     * @param joint_name The name of the joint associated with the motor.
     * @return True if the motor is ready, false otherwise.
     */
    bool isMotorReady(const std::string& joint_name) const;

    // Diagnostics

    /**
     * @brief Gets the position error of a joint.
     * @param joint_name The name of the joint.
     * @return The position error of the joint.
     */
    double getPositionError(const std::string& joint_name) const;

    /**
     * @brief Gets the time since the last update of a joint.
     * @param joint_name The name of the joint.
     * @return The time since the last update of the joint.
     */
    rclcpp::Duration getTimeSinceLastUpdate(const std::string& joint_name) const;

    /**
     * @brief Gets the last error message of a joint.
     * @param joint_name The name of the joint.
     * @return The last error message of the joint.
     */
    std::string getLastError(const std::string& joint_name) const;

    /**
     * @brief Clears the error of a joint.
     * @param joint_name The name of the joint.
     */
    void clearError(const std::string& joint_name);

private:
    rclcpp::Node::SharedPtr node_; /**< A shared pointer to the ROS 2 node. */
    std::shared_ptr<CANProtocol> can_protocol_; /**< A shared pointer to the CAN protocol. */
    
    std::map<std::string, JointConfig> joints_; /**< A map of joint names to joint configurations. */
    std::map<uint8_t, std::string> motor_to_joint_map_; /**< A map of motor IDs to joint names. */
    double position_tolerance_; /**< The position tolerance for joint control. */
    double velocity_tolerance_; /**< The velocity tolerance for joint control. */
    // Internal handlers

    /**
     * @brief Callback function for CAN messages.
     * @param msg A shared pointer to the CAN message.
     */
    void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);

    /**
     * @brief Processes a status response from a motor.
     * @param motor_id The ID of the motor.
     * @param data The data received from the motor.
     */
    void processStatusResponse(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief Processes an encoder response from a motor.
     * @param motor_id The ID of the motor.
     * @param data The data received from the motor.
     */
    void processEncoderResponse(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief Processes a velocity response from a motor.
     * @param motor_id The ID of the motor.
     * @param data The data received from the motor.
     */
    void processVelocityResponse(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief Processes an IO response from a motor.
     * @param motor_id The ID of the motor.
     * @param data The data received from the motor.
     */
    void processIOResponse(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief Processes an error response from a motor.
     * @param motor_id The ID of the motor.
     * @param data The data received from the motor.
     */
    void processErrorResponse(uint8_t motor_id, const std::vector<uint8_t>& data);

    /**
     * @brief Requests data from a motor.
     * @param motor_id The ID of the motor.
     */
    void requestMotorData(uint8_t motor_id);
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_HPP_