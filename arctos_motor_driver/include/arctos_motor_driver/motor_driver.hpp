#ifndef ARCTOS_MOTOR_DRIVER_HPP_
#define ARCTOS_MOTOR_DRIVER_HPP_

#include "arctos_motor_driver/motor_types.hpp"
#include "arctos_motor_driver/uart_protocol.hpp"
#include "serial/serial.h"
#include <map>
#include <memory>

/**
 * @file motor_driver.hpp
 * @brief This file contains the declaration of the MotorDriver class.
 */

namespace arctos_motor_driver {

// this is to define whether we need to convert from degrees to encoder steps or not.
#define ENCODER_CONVERSION_NEEDED  false

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

    void processUartMessage(void);
    // Joint management

    void setProtocol(std::shared_ptr<UartProtocol> protocol);
    
    /**
     * @brief Updates the states of the joints.
     */
    void updateJointStates();

    /**
     * @brief Adds a joint to the motor driver.
     * @param joint_name The name of the joint.
     * @param motor_id The ID of the motor.
     */
    void addJoint(const std::string& joint_name, uint8_t motor_id, std::string hardware_type, double gear_ratio = 1.0, bool inverted = false, bool inverted_feedback = false, double zero_position = 0.0, double lower_limit = 0.0, double upper_limit = 0.0);

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
     * @param acceleration Acceleration of the motor movement, default to 20 (in range 0 - 255)
     */
    void setJointPosition(const std::string& joint_name, double position, double acceleration = 20, double velocity = 100);

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
    double getJointPosition(const std::string& joint_name, bool convert_to_rad = true) const;

    /**
     * @brief Gets the velocity of a joint.
     * @param joint_name The name of the joint.
     * @return The velocity of the joint.
     */
    double getJointVelocity(const std::string& joint_name) const;
    
    // Motor control

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

    /**
     * @brief Write command to actuator, using buffer from each motors
     * @param 
     */
    void writeCommand();

private:
    rclcpp::Node::SharedPtr node_; /**< A shared pointer to the ROS 2 node. */
    std::shared_ptr<UartProtocol> uart_protocol_; /**< A shared pointer to the UART protocol. */
    
    std::map<std::string, JointConfig> joints_; /**< A map of joint names to joint configurations. */
    std::map<uint8_t, std::string> motor_to_joint_map_; /**< A map of motor IDs to joint names. */
    double position_tolerance_; /**< The position tolerance for joint control. */
    double velocity_tolerance_; /**< The velocity tolerance for joint control. */
    // Internal handlers

    // specify the size of encoder data for each joint
    const uint8_t ENCODER_SIZE = 1;
    // old buffer, use for comparison
    std::vector<std::vector<double>> pre_encoder_data_;

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
    void processEncoderResponse(uint8_t motor_id, const std::vector<double>& data);

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

    /**
     * @brief Checks if the encoder data has changed.
     * @param encoder_data The current encoder data.
     * @return True if the encoder data has changed, false otherwise.
     */
    bool isEncoderDataChanged(const std::vector<double>& encoder_data, const uint8_t motor_id) const;
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_DRIVER_HPP_