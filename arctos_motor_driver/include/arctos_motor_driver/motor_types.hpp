#ifndef ARCTOS_MOTOR_TYPES_HPP_
#define ARCTOS_MOTOR_TYPES_HPP_

#include <string>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

/**
 * @file motor_types.hpp
 * @brief This file contains the definition of various structures and enums related to motor control.
 */

namespace arctos_motor_driver {

/**
 * @brief Structure representing the status information of a motor.
 */
struct MotorStatus {
    bool is_enabled{false}; /**< Flag indicating if the motor is enabled. */
    bool is_protected{false}; /**< Flag indicating if the motor has the shaft protection protected. */
    bool is_calibrated{false}; /**< Flag indicating if the motor is calibrated. */
    bool is_homed{false}; /**< Flag indicating if the motor is homed. */
    bool is_error{false}; /**< Flag indicating if the motor has encountered an error. */
    uint8_t error_code{0}; /**< Error code of the motor. */
    std::string error_message{}; /**< Error message associated with the motor. */
    bool limit_switch_left{false}; /**< Flag indicating if the left limit switch is triggered. */
    bool limit_switch_right{false}; /**< Flag indicating if the right limit switch is triggered. */
    bool is_stalled{false}; /**< Flag indicating if the motor is stalled. */
};

/**
 * @brief Structure representing the configuration parameters of a motor.
 */
struct MotorParameters {
    uint8_t working_mode{2}; /**< Working mode of the motor. */
    uint16_t working_current{1600}; /**< Working current of the motor in mA. */
    uint8_t holding_current_percentage{50}; /**< Percentage of holding current. */
    uint16_t subdivisions{16}; /**< Subdivisions of the motor. */
    bool encoder_direction{false}; /**< Encoder direction of the motor. */
    bool protection_enabled{false}; /**< Flag indicating if protection is enabled. */
    uint32_t pulse_count{0}; /**< Pulse count of the motor. */
};

/**
 * @brief Structure representing the configuration of a joint.
 */
struct JointConfig {
    uint8_t motor_id{0}; /**< ID of the motor. */
    std::string joint_name; /**< Name of the joint. */
    std::string hardware_type; /**< Hardware type of the joint (MKS_42D or MKS_57D). */
    // Gear ratio
    double gear_ratio{1.0};
    
    // Current state
    double position{0.0}; /**< Current position of the joint. */
    double velocity{0.0}; /**< Current velocity of the joint. */
    double command_position{0.0}; /**< Commanded position of the joint. */
    double command_velocity{0.0}; /**< Commanded velocity of the joint. */
    double position_error{0.0}; /**< Position error of the joint. */
    
    // Timing info
    rclcpp::Time last_update; /**< Time of the last update. */
    rclcpp::Time last_command; /**< Time of the last command. */
    
    // Status and parameters
    MotorStatus status{}; /**< Status of the motor. */
    MotorParameters params{}; /**< Parameters of the motor. */
    
    // Motion limits
    double position_min{-M_PI}; /**< Minimum position of the joint. */
    double position_max{M_PI}; /**< Maximum position of the joint. */
    double velocity_max{50.0}; /**< Maximum velocity of the joint. */
    double acceleration_max{100.0}; /**< Maximum acceleration of the joint. */

    // Default constructor
    JointConfig() = default;


    // Legacy constructor for testing
    JointConfig(uint8_t id, const std::string& name) 
        : motor_id(id)
        , joint_name(name)
        , last_update(0, 0, RCL_SYSTEM_TIME)
        , last_command(0, 0, RCL_SYSTEM_TIME) {}
        
    // Constructor with id and name
    JointConfig(uint8_t id, const std::string& name, rclcpp::Clock::SharedPtr clock) 
        : motor_id(id)
        , joint_name(name)
        , last_update(clock->now())
        , last_command(clock->now()) {}
};

/**
 * @brief Enumeration representing the working modes of a motor.
 */
enum class MotorMode : uint8_t {
    CR_OPEN = 0, /**< Pulse interface Open mode (max 400 RPM). */
    CR_CLOSE = 1, /**< Pulse interface Close mode (max 1500 RPM). */
    CR_vFOC = 2, /**< Pulse interface FOC mode (max 3000 RPM). */
    SR_OPEN = 3, /**< Serial interface Open mode (max 400 RPM). */
    SR_CLOSE = 4, /**< Serial interface Close mode (max 1500 RPM). */
    SR_vFOC = 5 /**< Serial interface FOC mode (max 3000 RPM). */
};

/**
 * @brief Structure representing the CAN command codes from the MKS manual.
 */
struct CANCommands {
    // Query commands
    static constexpr uint8_t QUERY_MOTOR = 0xF1; /**< Query motor command. */
    
    // Read commands
    static constexpr uint8_t READ_ENCODER = 0x31; /**< Read encoder value (addition). */
    static constexpr uint8_t READ_VELOCITY = 0x32; /**< Read velocity command. */
    static constexpr uint8_t READ_PULSES = 0x33; /**< Read pulses command. */
    static constexpr uint8_t READ_IO = 0x34; /**< Read IO command. */
    static constexpr uint8_t READ_RAW_ENCODER = 0x35; /**< Read raw encoder command. */
    static constexpr uint8_t READ_ERROR = 0x39; /**< Read error command. */
    static constexpr uint8_t READ_ENABLE_STATE = 0x3A; /**< Read enable state command. */
    static constexpr uint8_t READ_SHAFT_PROTECTION_STATE = 0x3E; /**< Read protection command. */

    // Set/Control commands
    static constexpr uint8_t CALIBRATE = 0x80; /**< Calibrate command. */
    static constexpr uint8_t SET_WORKING_MODE = 0x82; /**< Set working mode command. */
    static constexpr uint8_t SET_CURRENT = 0x83; /**< Set current command. */
    static constexpr uint8_t SET_SUBDIVISIONS = 0x84; /**< Set subdivisions command. */
    static constexpr uint8_t SET_ENABLE_SETTINGS = 0x85; /**< Set enable settings command. */
    static constexpr uint8_t SET_DIRECTION = 0x86; /**< Set direction command. */
    static constexpr uint8_t ENABLE_PROTECTION = 0x88; /**< Enable protection command. */
    static constexpr uint8_t SET_HOME_PARAMS = 0x90; /**< Set home parameters command. */
    static constexpr uint8_t GO_HOME = 0x91; /**< Go home command. */
    static constexpr uint8_t SET_ZERO = 0x92; /**< Set zero command. */
    
    // Motion commands
    static constexpr uint8_t ENABLE_MOTOR = 0xF3; /**< Enable motor command. */
    static constexpr uint8_t RELATIVE_POSITION = 0xF4; /**< Relative position command. */
    static constexpr uint8_t ABSOLUTE_POSITION = 0xF5; /**< Absolute position command. */
    static constexpr uint8_t SPEED_CONTROL = 0xF6; /**< Speed control command. */
    static constexpr uint8_t EMERGENCY_STOP = 0xF7; /**< Emergency stop command. */
    static constexpr uint8_t POSITION_CONTROL = 0xFD; /**< Position control command. */
    static constexpr uint8_t ABSOLUTE_POSITION_PULSE = 0xFE; /**< Absolute position pulse command. */
};

/**
 * @brief Structure representing common conversion values for motors.
 */
struct MotorConstants {
    // Encoder constants
    static constexpr double ENCODER_STEPS = 16384.0;  // 0x4000 steps per revolution
    static constexpr double DEGREES_PER_REVOLUTION = 360.0;
    static constexpr double RADIANS_PER_REVOLUTION = 2.0 * M_PI;

    // Motor speed limits (RPM)
    static constexpr double MAX_RPM_OPEN = 400.0;    // OPEN mode max speed
    static constexpr double MAX_RPM_CLOSE = 1500.0;  // CLOSE mode max speed
    static constexpr double MAX_RPM_vFOC = 3000.0;   // vFOC mode max speed

    // Conversion factors
    static constexpr double RPM_TO_RADPS = M_PI / 30.0;     // RPM to rad/s (π/30)
    static constexpr double RADPS_TO_RPM = 30.0 / M_PI;     // rad/s to RPM (30/π)
    static constexpr double DEG_TO_RAD = M_PI / 180.0;      // degrees to radians
    static constexpr double RAD_TO_DEG = 180.0 / M_PI;      // radians to degrees
};

} // namespace arctos_motor_driver

#endif // ARCTOS_MOTOR_TYPES_HPP_