#include "arctos_motor_driver/motor_driver.hpp"
#include <cmath>
#include <string>
#include <sstream>

/**
 * @brief The MotorDriver class is responsible for controlling and managing multiple motors.
 * 
 * This class provides methods for adding and removing joints, setting joint position and velocity,
 * enabling and disabling motors, stopping motors, calibrating and homing motors, and retrieving
 * joint position and velocity. It also handles CAN message callbacks for processing motor responses
 * and updating joint states.
 */
namespace arctos_motor_driver {

/**
 * @brief Constructs a MotorDriver object.
 *
 * This constructor initializes a MotorDriver object with the given node.
 * It sets up a CAN message subscription and creates a timer for periodic status updates.
 *
 * @param node A shared pointer to the rclcpp::Node object.
 */
MotorDriver::MotorDriver(rclcpp::Node::SharedPtr node) 
    : node_(node),
      uart_protocol_(std::make_shared<UartProtocol>()) {
}

MotorDriver::~MotorDriver() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down motor driver, stopping all motors...");
    stopAllMotors();
}

void MotorDriver::setProtocol(std::shared_ptr<UartProtocol> protocol) {
    uart_protocol_ = protocol;
}
/**
 * @brief Adds a new joint to the motor driver.
 *
 * This function adds a new joint to the motor driver with the specified joint name and motor ID.
 * It checks if the joint already exists or if the motor ID is already in use by another joint.
 * If the joint is successfully added, it creates a new joint configuration and initializes the motor.
 *
 * @param joint_name The name of the joint to be added.
 * @param motor_id The ID of the motor associated with the joint.
 */
void MotorDriver::addJoint(const std::string& joint_name, uint8_t motor_id, std::string hardware_type, double gear_ratio, bool inverted,bool inverted_feedback, double zero_position, double lower_limit, double upper_limit) {
    // Check if joint already exists
    if (joints_.find(joint_name) != joints_.end()) {
        RCLCPP_WARN(node_->get_logger(), "Joint %s already exists", joint_name.c_str());
        return;
    }

    // Check if motor_id is already in use
    if (motor_to_joint_map_.find(motor_id) != motor_to_joint_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Motor ID %d is already in use by joint %s", 
                     motor_id, motor_to_joint_map_[motor_id].c_str());
        return;
    }

    // Validate gear ratio
    if (gear_ratio <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid gear ratio %.2f for joint %s", gear_ratio, joint_name.c_str());
        return;
    }

    // Create and insert new joint
    joints_.insert({joint_name, JointConfig(motor_id, joint_name, node_->get_clock())});
    joints_[joint_name].hardware_type = hardware_type;
    joints_[joint_name].gear_ratio = gear_ratio;
    joints_[joint_name].inverted = inverted;
    joints_[joint_name].inverted_feedback = inverted_feedback;
    joints_[joint_name].zero_position = zero_position;
    joints_[joint_name].lower_limit = lower_limit;
    joints_[joint_name].upper_limit = upper_limit;
    motor_to_joint_map_[motor_id] = joint_name;

    joints_[joint_name].last_update = node_->get_clock()->now();  // Initialize timestamp
    // add a vector of "encoder" data for each joint, emplace_back() at the pre_encoder_data
    // make sure that the datatype used in emplace_back here match the definition!
    pre_encoder_data_.emplace_back(std::vector<double>(1, 0));

    RCLCPP_INFO(node_->get_logger(), "Added joint %s with motor ID %d and gear ratio %.2f:1", 
                joint_name.c_str(), motor_id, gear_ratio);

    // Log current joints and motor-to-joint map
    RCLCPP_DEBUG(node_->get_logger(), "Current Joints:");
    for (const auto& joint : joints_) {
        RCLCPP_DEBUG(node_->get_logger(), "  Joint Name: %s, Motor ID: %d, Gear Ratio: %.2f:1",
                    joint.first.c_str(), joint.second.motor_id, joint.second.gear_ratio);
    }

    RCLCPP_DEBUG(node_->get_logger(), "Motor-to-Joint Map:");
    for (const auto& entry : motor_to_joint_map_) {
        RCLCPP_DEBUG(node_->get_logger(), "  Motor ID %d -> Joint %s", entry.first, entry.second.c_str());
    }
}

/**
 * @brief Removes a joint from the MotorDriver.
 * 
 * This function removes a joint from the MotorDriver based on the provided joint name.
 * If the joint is found, the associated motor is stopped, and the joint is removed from the internal data structures.
 * 
 * @param joint_name The name of the joint to be removed.
 */
void MotorDriver::removeJoint(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it != joints_.end()) {
        uint8_t motor_id = it->second.motor_id;
        stopMotor(joint_name);
        motor_to_joint_map_.erase(motor_id);
        joints_.erase(it);
        RCLCPP_INFO(node_->get_logger(), "Removed joint %s", joint_name.c_str());
    }
}

/**
 * @brief Sets the position of the joint.
 *
 * This function sets the desired position of the joint.
 * application only care about what joint. 
 * the driver has to handle logic how to successfully rotate the joint.
 *
 * @param position The desired position of the joint in radians.
 * @param acceleration The desired acceleration in degrees/s^2.
 */
void MotorDriver::setJointPosition(const std::string& joint_name, double position, double acceleration, double velocity) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    auto& joint = it->second;

    if (joint.inverted) {
        position = -position;
    }

    // Scale the position by the gear ratio
    double motor_position = position * joint.gear_ratio;

    // Convert motor position from radians to degrees
    double motor_position_deg = motor_position * MotorConstants::RAD_TO_DEG;

#if ENCODER_CONVERSION_NEEDED
    // Convert degrees to encoder counts
    int32_t encoder_counts = static_cast<int32_t>(
        (motor_position_deg * MotorConstants::ENCODER_STEPS) / MotorConstants::DEGREES_PER_REVOLUTION
    );
#endif

    RCLCPP_INFO(node_->get_logger(), "Setting joint %s position to (%s) %.2f radians (%.2f degrees on motor protactor) with gear ratio %.2f:1",
                joint_name.c_str(), joint.inverted ? "inverted" : "normal", position, motor_position_deg, joint.gear_ratio);
#if ENCODER_CONVERSION_NEEDED
    RCLCPP_INFO(node_->get_logger(), "Calculated encoder counts: %d", encoder_counts);
#endif

    // minimum speed is 30 to prevent sending speed = 0, which will stop the motor!
    uint16_t speed = static_cast<uint16_t>(std::clamp(velocity, 30.0, 3000.0));  
    uint8_t acc_value = static_cast<uint8_t>(std::clamp(acceleration, 10.0, 255.0));

    // Write to the data buffer, ready to send.
    joint.command_velocity = speed;
    joint.command_acceleration = acc_value;
    joint.command_position = motor_position_deg;
}

/**
 * @brief Write command to actuator, using buffer from each motors
 * @param 
 */
void MotorDriver::writeCommand() {
    std::string command = "";
    std::vector<double> commandPositions;
    for (auto& [joint_name, joint] : joints_) {
        // Send CAN command
        // std::vector<uint8_t> data1;
        // can_protocol_->sendFrame(joint.motor_id, data1);
        command.append(std::to_string(joint.command_position) + ";");
        commandPositions.push_back(joint.command_position);
        joint.last_command = node_->get_clock()->now();
    }
    uart_protocol_->sendPosition(commandPositions);
    RCLCPP_INFO(node_->get_logger(), "Write command to actuator: %s", command.c_str());
}


/**
 * Sets the velocity of a joint in the motor driver.
 * 
 * @param joint_name The name of the joint.
 * @param velocity The desired velocity in rad/s.
 */
void MotorDriver::setJointVelocity(const std::string& joint_name, double velocity) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    auto& joint = it->second;

    if (joint.inverted) {
        velocity = -velocity;
    }

    // Check velocity limits
    // TODO: clamp velocity to max velocity and support -max_velocity, max_velocity
    if (std::abs(velocity) > joint.velocity_max) {
        RCLCPP_WARN(node_->get_logger(), 
                    "Velocity %.2f for joint %s exceeds limit %.2f",
                    velocity, joint_name.c_str(), joint.velocity_max);
        velocity = std::copysign(joint.velocity_max, velocity);
    }

    // Convert rad/s to RPM
    double rpm = std::abs(velocity) * MotorConstants::RADPS_TO_RPM;
    
    // Determine max speed based on working mode
    double max_speed;
    
    // Clamp speed to mode-specific max
    rpm = std::min(rpm, max_speed);
    
    // Prepare velocity command
    uint8_t direction = velocity >= 0 ? 0x00 : 0x80;
    uint16_t speed = static_cast<uint16_t>(rpm);
    
    joint.command_velocity = velocity;
    joint.last_command = node_->get_clock()->now();
}

/**
 * @brief Retrieves the position of a joint.
 * 
 * This function returns the position of the specified joint. If the joint is not found, an error message is logged and 0.0 is returned.
 * 
 * @param joint_name The name of the joint to retrieve the position from.
 * @return The position of the joint.
 */
double MotorDriver::getJointPosition(const std::string& joint_name, bool convert_to_rad) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return 0.0;
    }

    RCLCPP_DEBUG(node_->get_logger(), "Retrieved position for joint %s: %.3f", joint_name.c_str(), it->second.position);
    if(convert_to_rad)
        return it->second.position * MotorConstants::DEG_TO_RAD;
    else
        return it->second.position;
    
}

/**
 * @brief Get the velocity of a specific joint.
 * 
 * This function retrieves the velocity of a joint specified by its name.
 * If the joint is not found, an error message is logged and a default value of 0.0 is returned.
 * 
 * @param joint_name The name of the joint.
 * @return The velocity of the joint.
 */
double MotorDriver::getJointVelocity(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return 0.0;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Retrieved velocity for joint %s: %.3f", joint_name.c_str(), it->second.velocity);
    return it->second.velocity;
}

/**
 * Stops the motor associated with the given joint name.
 *
 * @param joint_name The name of the joint.
 */
void MotorDriver::stopMotor(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    // Send emergency stop command
    
    // Reset command values
    it->second.command_velocity = 0.0;
    it->second.command_position = it->second.position;
}

/**
 * @brief Stops all motors.
 *
 * This function stops all motors controlled by the MotorDriver class.
 * It iterates over all joints and calls the stopMotor function for each joint.
 */
void MotorDriver::stopAllMotors() {
    for (const auto& joint : joints_) {
        stopMotor(joint.first);
    }
}

/**
 * @brief Checks if a motor is ready.
 *
 * This function checks if a motor with the specified joint name is ready. It looks for the joint name in the `joints_` map and returns `true` if the joint is found and meets the following conditions:
 * - The motor is enabled
 * - The motor is calibrated
 * - The motor is homed
 * - The motor is not in an error state
 * - The motor is not stalled
 *
 * If the joint is not found in the `joints_` map, an error message is logged and `false` is returned.
 *
 * @param joint_name The name of the joint to check.
 * @return `true` if the motor is ready, `false` otherwise.
 */
bool MotorDriver::isMotorReady(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return false;
    }

    const auto& status = it->second.status;
    return status.is_enabled && 
           status.is_calibrated && 
           status.is_homed && 
           !status.is_error && 
           !status.is_stalled;
}

/**
 * @brief Updates the joint states by sending requests for encoder position, velocity, and IO status.
 * 
 * This function iterates over each joint in the `joints_` map and sends CAN frames to request the encoder position,
 * velocity, and IO status (including limit switches) for each joint's motor. The requests are sent using the `can_protocol_`
 * object.
 * 
 * @note This function assumes that the `joints_` map has been populated with valid joint objects and that the `can_protocol_`
 * object has been properly initialized.
 */
void MotorDriver::updateJointStates() {
    // RCLCPP_WARN(node_->get_logger(), "[updateJointStates] Starting joint state update cycle");

    auto current_time = node_->get_clock()->now();
    if (joints_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[updateJointStates] No joints registered for state updates");
        return;
    }
    bool commandUpdate = false;
    bool statusUpdate = false;

    // Iterate through all joints
    for (auto& [joint_name, joint] : joints_) {
        // last_command is updated if we send any command to the bus
        auto time_since_last_command = current_time - joint.last_command;
        // last_update is updated if we process and received data
        auto time_since_last_update = current_time - joint.last_update;
        commandUpdate = true;
        statusUpdate = true;

        // Stop movement if joint is still moving but no updates received in 2 seconds
        if (joint.status.is_moving && time_since_last_update.seconds() > 2.0) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "[updateJointStates] Joint '%s' hasn't updated for %.2f sec, forcing is_moving = false",
                        joint_name.c_str(), time_since_last_update.seconds());
            joint.status.is_moving = false;
            joint.velocity = 0.0;  // Ensure velocity is reset
            statusUpdate = false;
        }

        if (time_since_last_update.seconds() > 0.5) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "[updateJointStates] No status update for joint '%s' (%.2f seconds). Stopping requests.",
                        joint_name.c_str(), time_since_last_update.seconds());
            statusUpdate = false;
        }

        // Set velocity to 0.0 if no recent commands (0.9 sec threshold)
        if (time_since_last_command.seconds() > 0.9) {  
            RCLCPP_DEBUG(node_->get_logger(),
                         "[updateJointStates] No recent command for joint '%s' (%.2f seconds). Setting velocity to 0",
                         joint_name.c_str(), time_since_last_command.seconds());
            joint.velocity = 0.0;  
            commandUpdate = false;
        }

        /* if there's no status or command update for a long time, stop requesting data.*/
        if (statusUpdate || commandUpdate) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "[updateJointStates] Requesting new state since %s || %s for joint '%s' (motor_id: %d)",
                        statusUpdate ? "status update" : "false",
                        commandUpdate ? "command update" : "false",
                        joint_name.c_str(), joint.motor_id);

            try {
                requestMotorData(joint.motor_id);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(),
                            "[updateJointStates] Error updating state for joint '%s': %s",
                            joint_name.c_str(), e.what());
            }
        }

        // Request new state only if recent command exists
        
    }

    RCLCPP_DEBUG(node_->get_logger(), "[updateJointStates] Completed joint state update cycle");
}


// void MotorDriver::processCANMessage(const can_msgs::msg::Frame::SharedPtr msg) {
//     std::stringstream encoder_data;
//     for (uint8_t i = 0; i < msg->dlc; i++) {
//         if (i == 0) {
//             encoder_data << "[ ";
//         } 
//         encoder_data << std::hex << static_cast<int>(msg->data[i]) << " ";
//         if (i == msg->dlc-1) {
//             encoder_data << "]"; 
//         }
//     }
//     RCLCPP_INFO(node_->get_logger(), "Rx Data for %d: %s", msg->id, encoder_data.str().c_str());

//     canMessageCallback(msg);
// }

void MotorDriver::processUartMessage() {
    std::string message;
    std::vector<double> decodedPositions;
    do {
        message = uart_protocol_->getFromBuffer();
        if (message != "") {
            decodedPositions = uart_protocol_->decodeMessage(message);
            if (decodedPositions.size() != joints_.size()) 
            {
                RCLCPP_ERROR(node_->get_logger(), "The number of decoded joints does not match with the configured joints!");
            }
            else
            {
                
                for (auto& [joint_name, joint] : joints_) 
                {
                    std::vector<double> decodedPosition = {decodedPositions[joint.motor_id - 1]};
                    processEncoderResponse(joint.motor_id, decodedPosition);      
                    
                }
            }
            
        }
    }
    while (message != "");
}

/**
 * @brief Callback function for CAN messages.
 * 
 * This function is called when a CAN message is received. It processes the message based on the command type and the motor ID.
 * 
 * @param msg The CAN message received.
 */
// void MotorDriver::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg) {
//     RCLCPP_DEBUG(node_->get_logger(), "CAN message received - ID: %d, Command: 0x%02X", 
//                 msg->id, msg->data[0]);
//     auto motor_it = motor_to_joint_map_.find(msg->id);
//     if (motor_it == motor_to_joint_map_.end()) {
//         RCLCPP_DEBUG(node_->get_logger(), "Received message for unknown motor ID: %d", msg->id);
//         return;
//     }

//     std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
    
//     if (data.empty()) {
//         RCLCPP_WARN(node_->get_logger(), "Received empty CAN message");
//         return;
//     }

//     switch(data[0]) {
//         case CANCommands::READ_ENCODER:
//             if (msg->dlc < 8) {
//                 RCLCPP_WARN(node_->get_logger(), "Invalid CAN message response length for READ_ENCODER");
//                 return;
//             } else {
//                 RCLCPP_INFO(node_->get_logger(), "Processing encoder response");
//                 processEncoderResponse(msg->id, data);
//             }
//             break;
            
//         case CANCommands::READ_VELOCITY:
//             RCLCPP_INFO(node_->get_logger(), "Processing velocity response");
//             processVelocityResponse(msg->id, data);
//             break;
            
//         case CANCommands::READ_IO:
//             RCLCPP_INFO(node_->get_logger(), "Processing IO response");
//             processIOResponse(msg->id, data);
//             break;
            
//         case CANCommands::ABSOLUTE_POSITION:
//         case CANCommands::CALIBRATE:
//         case CANCommands::GO_HOME:
//         case CANCommands::ENABLE_MOTOR:
//         case CANCommands::SET_ZERO_POSITION:
//         case CANCommands::ENABLE_SHAFT_PROTECTION:
//             RCLCPP_INFO(node_->get_logger(), "Processing status response");
//             processStatusResponse(msg->id, data);
//             break;
            
//         default:
//             RCLCPP_INFO(node_->get_logger(), "Received unhandled command: 0x%02X", data[0]);
//             break;
//     }
// }

/**
 * @brief Process the response from the encoder for a specific motor.
 *
 * This function extracts the encoder data from the received data and updates the joint state accordingly.
 * It calculates the position error by subtracting the current joint position from the commanded position.
 *
 * @param motor_id The ID of the motor.
 * @param data The received data from the encoder.
 */
void MotorDriver::processEncoderResponse(uint8_t motor_id, const std::vector<double>& data) {
    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) {
        RCLCPP_WARN(node_->get_logger(), "No joint found for motor ID: %d", motor_id);
        return;
    }

    const std::string& joint_name = it->second;
    auto& joint = joints_[joint_name];

    // if the encoder response is exactly the same as previous encoder data, no need to process.
    if (isEncoderDataChanged(data, motor_id) == false) {
        RCLCPP_INFO(node_->get_logger(), "No new data for %d", motor_id);
        return;
    } else {
        pre_encoder_data_[motor_id-1] = data;
    }

    try {
        // **Log raw data for debugging**
        RCLCPP_DEBUG(node_->get_logger(), "Raw Encoder Data for %s:", joint_name.c_str());
        for (size_t i = 0; i < data.size(); i++) {
            RCLCPP_DEBUG(node_->get_logger(), "Byte %zu: 0x%02X", i, data[i]);
        }

        // **Get the decoded data as the first element**
        double motor_angle_deg = data.front();
        RCLCPP_DEBUG(node_->get_logger(), "Decoded motor angle (degrees): %.2f", motor_angle_deg);

        // **Check for invalid values (NaN/Inf)**
        if (!std::isfinite(motor_angle_deg)) {
            RCLCPP_WARN(node_->get_logger(), "Invalid motor angle detected for joint %s. Skipping update.", joint_name.c_str());
            return;
        }

        // **Sanity check for absurd values**
        constexpr double MAX_MOTOR_DEGREES = 360.0 * 1000;  // 1000 revolutions
        if (motor_angle_deg > MAX_MOTOR_DEGREES || motor_angle_deg < -MAX_MOTOR_DEGREES) {
            RCLCPP_WARN(node_->get_logger(), "Discarding out-of-range motor angle: %.2f degrees for motor ID %d", 
                        motor_angle_deg, motor_id);
            return;
        }

        // **Convert motor angle to joint angle**
        double joint_angle_deg = motor_angle_deg / joint.gear_ratio;

        // **Apply inversion_feedback BEFORE zero position offset**
        if (joint.inverted_feedback) {
            joint_angle_deg = -joint_angle_deg;
        }

        RCLCPP_DEBUG(node_->get_logger(), "Final computed joint angle: %.3f rad", joint_angle_deg);

        // // **Filter sudden jumps using a moving average**
        // constexpr double FILTER_ALPHA = 0.3;
        // joint.position = FILTER_ALPHA * joint_angle_deg + (1.0 - FILTER_ALPHA) * joint.position;

        // **Print Joint Limits for Debugging**
        RCLCPP_DEBUG(node_->get_logger(), "Joint %s: Min = %.3f, Max = %.3f, Current = %.3f", 
                    joint_name.c_str(), joint.position_min, joint.position_max, joint.position);

        joint.position = joint_angle_deg;
        // **Check if joint limits are valid**
        constexpr double TOLERANCE = 0.1;
        if (joint.position < (joint.position_min - TOLERANCE) || joint.position > (joint.position_max + TOLERANCE)) {
            RCLCPP_WARN(node_->get_logger(), "Ignoring out-of-bounds encoder value %.3f rad for joint %s (limits: %.3f to %.3f)",
                        joint.position, joint_name.c_str(), joint.position_min, joint.position_max);
            return;
        }

        // **Update Joint State**
        joint.position_error = joint.command_position - joint.position;

        // **Apply Deadband Filtering (to remove tiny errors)**
        constexpr double POSITION_DEADBAND = 0.0001;
        if (std::abs(joint.position) < POSITION_DEADBAND) {
            joint.position = 0.0;
        }
        RCLCPP_INFO(node_->get_logger(), "Updated motor %d (%s) position: %.2f rad", 
                    joint.motor_id, joint.inverted_feedback ? "inverted feedback" : "normal", joint.position);

        joint.last_update = node_->get_clock()->now();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error processing encoder response: %s", e.what());
    }
}

/**
 * @brief Checks if the encoder data has changed.
 * @param encoder_data The current encoder data. expect a 6-byte vector
 * @return True if the encoder data has changed, false otherwise.
 */
bool MotorDriver::isEncoderDataChanged(const std::vector<double>& encoder_data, const uint8_t motor_id) const {
    if (motor_id < 1 || motor_id > pre_encoder_data_.size()) {
        RCLCPP_WARN(node_->get_logger(), "Invalid motor ID or wrong encoder data size: %d", motor_id);
        return false;
    }
    // Compare first bytes
    for (uint8_t i = 0; i < ENCODER_SIZE; ++i) {
        if (encoder_data[i] != pre_encoder_data_[motor_id - 1][i]) {
            return true;
        }
    }
    return false;
}


/**
 * @brief Process the velocity response received from the motor.
 * 
 * This function extracts the velocity data from the received data and updates the joint state accordingly.
 * 
 * @param motor_id The ID of the motor.
 * @param data The vector containing the received data.
 */
void MotorDriver::processVelocityResponse(uint8_t motor_id, const std::vector<uint8_t>& data) {
    if (data.size() < 3) return;

    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) return;

    auto& joint = joints_[it->second];

    // Extract velocity data (bytes 1-2)
    std::vector<uint8_t> velocity_data(data.begin() + 1, data.begin() + 3);

    try {
        double rpm = 0;
        
        // Apply inversion if necessary
        if (joint.inverted) {
            rpm = -rpm;
        }

        // Convert RPM to rad/s
        double rad_per_sec = rpm * MotorConstants::RPM_TO_RADPS;

        // **Clamp velocity within [-velocity_max, velocity_max]**
        double max_velocity = joint.velocity_max;
        if (std::abs(rad_per_sec) > max_velocity) {
            RCLCPP_WARN(node_->get_logger(), 
                        "Velocity out of range for joint %s: %.3f rad/s (clamped to %.3f rad/s)", 
                        joint.joint_name.c_str(), rad_per_sec, std::copysign(max_velocity, rad_per_sec));
        }

        // Apply clamping
        rad_per_sec = std::clamp(rad_per_sec, -max_velocity, max_velocity);

        // Store the validated velocity
        joint.velocity = rad_per_sec;
        joint.last_update = node_->get_clock()->now();

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error decoding velocity: %s", e.what());
    }
}

/**
 * @brief Process the I/O response for a specific motor.
 *
 * This function updates the limit switch states for the motor based on the received data.
 * It checks the size of the data vector and returns early if it is less than 2.
 * The function then retrieves the joint associated with the motor and updates its limit switch states
 * based on the IO status byte in the data.
 * Finally, it updates the last update time for the joint.
 *
 * @param motor_id The ID of the motor.
 * @param data The vector containing the received data.
 */
void MotorDriver::processIOResponse(uint8_t motor_id, const std::vector<uint8_t>& data) {
    if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "IO response too short for motor ID: %d", motor_id);
        return;
    }

    // Check if motor_id maps to a joint
    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) {
        RCLCPP_WARN(node_->get_logger(), "No joint found for motor ID: %d", motor_id);
        return;
    }

    auto& joint = joints_[it->second];

    // Extract IO status byte
    uint8_t io_status = data[1];

    // Reset the limit switches before applying new state
    // // TODO: Reset the joint status for both limit switches before setting them
    joint.status.limit_switch_left = false;
    joint.status.limit_switch_right = false;

    // Decode limit switch states based on hardware type
    // TODO: Fix the logic
    // TODO: When one home switch is used, set the other limit switch as the opposite of the home switch
    if (joint.hardware_type == "MKS_42D") {
        // Remapped ports for MKS 42D
        joint.status.limit_switch_left = (io_status & (1 << 2)) != 0;  // IN_2 (Bit 2, Dir)
        joint.status.limit_switch_right = (io_status & (1 << 3)) != 0; // IN_1 (Bit 3, En)
    } else if (joint.hardware_type == "MKS_57D") {
        // For MKS 57D (assuming active low: 0 means triggered)
        joint.status.limit_switch_left = (io_status & (1 << 1)) == 0;  // IN_1 on Bit1
        joint.status.limit_switch_right = (io_status & (1 << 0)) == 0; // IN_2 on Bit0
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown hardware type for motor ID: %d", motor_id);
        return;
    }

    // TODO: Implement homing status based on limit switch
    // Update homing status based on home endstop limit switch
    // if (joint.status.limit_switch_left) {  // Assuming left limit switch is the home switch
    //     joint.status.is_homed = true;
    // } else {
    //     joint.status.is_homed = false;
    // }

    // Update joint timestamp
    joint.last_update = node_->get_clock()->now();

    // Debug log for IO status
    RCLCPP_INFO(node_->get_logger(),
                "Processed IO response for motor ID: %d, IO Status: 0x%02X, "
                "Limit Switch Left: %s, Limit Switch Right: %s",
                motor_id, io_status,
                joint.status.limit_switch_left ? "TRIGGERED" : "NOT TRIGGERED",
                joint.status.limit_switch_right ? "TRIGGERED" : "NOT TRIGGERED");
}

/**
 * @brief Process the status response received for a motor.
 *
 * This function is responsible for processing the status response received for a motor. It updates the status of the corresponding joint based on the received data.
 *
 * @param motor_id The ID of the motor.
 * @param data The vector containing the status response data.
 */
void MotorDriver::processStatusResponse(uint8_t motor_id, const std::vector<uint8_t>& data) {
    RCLCPP_DEBUG(node_->get_logger(), "processStatusResponse - data size: %zu, data[0]: %d (0x%02X), data[1]: %d (0x%02X)", 
                 data.size(), data[0], data[0], data[1], data[1]);

    if (data.size() < 2) {
        RCLCPP_WARN(node_->get_logger(), "Status response too short: %zu bytes", data.size());
        return;
    }
    
    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) {
        RCLCPP_WARN(node_->get_logger(), "No joint found for motor ID: %d", motor_id);
        return;
    }
    
    auto& joint = joints_[it->second];
    uint8_t cmd = data[0];
    uint8_t status = data[1];

    switch(cmd) {
        case CANCommands::GO_HOME:
            RCLCPP_DEBUG(node_->get_logger(), "Inside GO_HOME case - Status: %d (0x%02X)", status, status);
            switch(status) {
                case 0:  // Homing failed
                    joint.status.is_homed = false;
                    joint.status.is_error = true;
                    joint.status.error_message = "Homing failed";
                    break;
                case 1:  // Homing in progress
                    joint.status.is_homed = false;
                    joint.status.is_error = false;
                    joint.status.error_message.clear();
                    joint.last_command = node_->get_clock()->now();
                    joint.status.is_moving = true;  // ✅ Ensure moving state is set
                    break;
                case 2:  // Homing successful
                    joint.status.is_homed = true;
                    joint.status.is_error = false;
                    joint.status.error_message.clear();
                    joint.last_command = node_->get_clock()->now();
                    joint.status.is_moving = false;  // ✅ Ensure movement stops
                    break;
            }
            break;
            
        case CANCommands::ENABLE_MOTOR:
            joint.status.is_enabled = (status == 1);
            break;

        case CANCommands::CALIBRATE:
            joint.status.is_calibrated = (status == 1);
            break;

        case CANCommands::ABSOLUTE_POSITION:
            RCLCPP_INFO(node_->get_logger(), "Inside ABSOLUTE_POSITION case - Status: %d (0x%02X)", status, status);
            switch(status) {
                case 1:  // Movement started
                    joint.status.is_error = false;
                    joint.status.is_moving = true;  // ✅ Set movement flag
                    joint.status.error_message.clear();
                    joint.last_command = node_->get_clock()->now();
                    break;
                case 2:  // Movement completed
                    RCLCPP_INFO(node_->get_logger(), "Move to absolute position completed");
                    joint.status.is_moving = false;  // ✅ Ensure movement stops
                    break;
                case 3:  // Movement partially completed (Endstop reached)
                    RCLCPP_INFO(node_->get_logger(), "Move to absolute position partially completed (Endstop limit reached)");
                    joint.status.is_moving = false;  // ✅ Ensure movement stops
                    break;
            }
            break;
    }

    joint.last_update = node_->get_clock()->now();
}

/**
 * @brief Sets the limits for a specific joint.
 * 
 * This function sets the position, velocity, and acceleration limits for a specific joint.
 * If the joint is not found, an error message is logged and the function returns.
 * 
 * @param joint_name The name of the joint.
 * @param pos_min The minimum position limit for the joint.
 * @param pos_max The maximum position limit for the joint.
 * @param vel_max The maximum velocity limit for the joint.
 * @param acc_max The maximum acceleration limit for the joint.
 */
void MotorDriver::setJointLimits(const std::string& joint_name, 
                                double pos_min, double pos_max,
                                double vel_max, double acc_max) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    // Validate limits
    if (pos_min > pos_max) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Invalid position limits for joint %s: min (%.2f) > max (%.2f)",
                    joint_name.c_str(), pos_min, pos_max);
        return;
    }

    // Update joint limits
    it->second.position_min = pos_min;
    it->second.position_max = pos_max;

    if (vel_max < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Invalid velocity limit for joint %s: %.2f",
                    joint_name.c_str(), vel_max);
        return;
    }

    it->second.velocity_max = vel_max;

    if (acc_max < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Invalid acceleration limit for joint %s: %.2f",
                    joint_name.c_str(), acc_max);
        return;
    }

    it->second.acceleration_max = acc_max;

    RCLCPP_INFO(node_->get_logger(), 
                "Updated limits for joint %s: pos=[%.2f, %.2f], vel=%.2f, acc=%.2f",
                joint_name.c_str(), pos_min, pos_max, vel_max, acc_max);
}

/**
 * @brief Get the position error for a specific joint.
 * 
 * This function retrieves the position error for a given joint name.
 * If the joint is not found, an error message is logged and 0.0 is returned.
 * 
 * @param joint_name The name of the joint.
 * @return The position error of the joint.
 */
double MotorDriver::getPositionError(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return 0.0;
    }
    return it->second.position_error;
}

/**
 * @brief Get the time elapsed since the last update for a specific joint.
 * 
 * This function returns the duration between the current time and the last update time
 * for the specified joint. If the joint is not found, an error message is logged and
 * a zero duration is returned.
 * 
 * @param joint_name The name of the joint.
 * @return The duration between the current time and the last update time for the joint.
 */
rclcpp::Duration MotorDriver::getTimeSinceLastUpdate(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return rclcpp::Duration(0, 0);
    }
    
    // Use the same clock source
    auto clock = node_->get_clock();
    return clock->now() - it->second.last_update;
}

/**
 * Retrieves the last error message for a specific joint.
 *
 * @param joint_name The name of the joint.
 * @return The last error message for the specified joint. If the joint is not found, "Joint not found" is returned.
 */
std::string MotorDriver::getLastError(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        return "Joint not found";
    }
    return it->second.status.error_message;
}

/**
 * @brief Clears the error status of a specific joint.
 * 
 * This function clears the error status of the joint specified by `joint_name`.
 * If the joint is found in the `joints_` map, its error status is set to false,
 * the error message is cleared, and the error code is set to 0.
 * 
 * @param joint_name The name of the joint to clear the error status for.
 */
void MotorDriver::clearError(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it != joints_.end()) {
        it->second.status.is_error = false;
        it->second.status.error_message.clear();
        it->second.status.error_code = 0;
    }
}

/**
 * @brief Retrieves the status of a motor.
 * 
 * This function returns the status of a motor specified by the given joint name.
 * If the joint name is not found in the internal map of joints, a std::runtime_error
 * is thrown with an error message indicating that the joint was not found.
 * 
 * @param joint_name The name of the joint to retrieve the status for.
 * @return The status of the motor.
 * @throws std::runtime_error if the joint name is not found.
 */
MotorStatus MotorDriver::getMotorStatus(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        throw std::runtime_error("Joint not found: " + joint_name);
    }
    return it->second.status;
}

/**
 * Retrieves the motor parameters for a given joint.
 *
 * @param joint_name The name of the joint.
 * @return The motor parameters for the specified joint.
 * @throws std::runtime_error if the joint is not found.
 */
MotorParameters MotorDriver::getMotorParameters(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        throw std::runtime_error("Joint not found: " + joint_name);
    }
    return it->second.params;
}

/**
 * Requests motor data for a specific motor.
 *
 * @param motor_id The ID of the motor to request data from.
 */
void MotorDriver::requestMotorData(uint8_t motor_id) {
    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) {
        RCLCPP_WARN(node_->get_logger(), "No joint found for motor ID: %d", motor_id);
        return;
    }

    auto& joint = joints_[it->second];
    RCLCPP_DEBUG(node_->get_logger(), "Joint %s (Motor ID: %d) is_moving: %s", 
            joint.joint_name.c_str(), motor_id, joint.status.is_moving ? "true" : "false");

    // Request sequence of motor data
    std::vector<std::vector<uint8_t>> requests = {
        {CANCommands::READ_ENCODER},  // Encoder position
        // {CANCommands::READ_IO}        // IO Status, probably needed when use the sensor
    };

    // ✅ Only add READ_VELOCITY if the joint is moving
    if (joint.status.is_moving) {
        // requests.push_back({CANCommands::READ_VELOCITY});
    }

    for (const auto& request : requests) {
        // can_protocol_->sendFrame(motor_id, request);
        // Add small delay between requests to prevent flooding
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/**
 * @brief Process the error response received from the motor.
 *
 * This function is responsible for processing the error response received from the motor.
 * It updates the error status and error message of the corresponding joint based on the error code received.
 * If the error code indicates an error, it logs an error message with the joint name and the error message.
 *
 * @param motor_id The ID of the motor.
 * @param data The error response data received from the motor.
 */ 
void MotorDriver::processErrorResponse(uint8_t motor_id, const std::vector<uint8_t>& data) {
    if (data.size() < 2) return;
    
    auto& joint = joints_[motor_to_joint_map_[motor_id]];
    
    joint.status.error_code = data[1];
    joint.status.is_error = (data[1] != 0);
    
    if (joint.status.is_error) {
        // Map error codes to messages
        switch(data[1]) {
            case 0x01:
                joint.status.error_message = "Motor stalled";
                break;
            case 0x02:
                joint.status.error_message = "Over temperature";
                break;
            case 0x03:
                joint.status.error_message = "Position error too large";
                break;
            default:
                joint.status.error_message = "Unknown error: " + std::to_string(data[1]);
        }
        
        RCLCPP_ERROR(node_->get_logger(), 
                    "Error detected on joint %s: %s", 
                    joint.joint_name.c_str(), 
                    joint.status.error_message.c_str());
    } else {
        joint.status.error_message.clear();
    }
}

} // namespace arctos_motor_driver