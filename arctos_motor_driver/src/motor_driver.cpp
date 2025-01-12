#include "arctos_motor_driver/motor_driver.hpp"
#include <cmath>

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
      can_protocol_(std::make_shared<CANProtocol>(node)) {}

MotorDriver::~MotorDriver() {
    RCLCPP_INFO(node_->get_logger(), "Shutting down motor driver, stopping all motors...");
    stopAllMotors();
}

void MotorDriver::setCAN(std::shared_ptr<CANProtocol> can_protocol) {
    can_protocol_ = can_protocol;
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
void MotorDriver::addJoint(const std::string& joint_name, uint8_t motor_id, std::string hardware_type, double gear_ratio) {
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
    motor_to_joint_map_[motor_id] = joint_name;
    joints_[joint_name].last_update = node_->get_clock()->now();  // Initialize timestamp

    RCLCPP_INFO(node_->get_logger(), "Added joint %s with motor ID %d and gear ratio %.2f:1", 
                joint_name.c_str(), motor_id, gear_ratio);

    // Log current joints and motor-to-joint map
    RCLCPP_INFO(node_->get_logger(), "Current Joints:");
    for (const auto& joint : joints_) {
        RCLCPP_INFO(node_->get_logger(), "  Joint Name: %s, Motor ID: %d, Gear Ratio: %.2f:1",
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
 *
 * @param position The desired position of the joint in radians.
 * @param acceleration The desired acceleration in degrees/s^2.
 */
void MotorDriver::setJointPosition(const std::string& joint_name, double position, double acceleration) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    auto& joint = it->second;

    // Scale the position by the gear ratio
    double motor_position = position * joint.gear_ratio;

    // Convert motor position from radians to degrees
    double motor_position_deg = motor_position * MotorConstants::RAD_TO_DEG;

    // Convert degrees to encoder counts
    int32_t encoder_counts = static_cast<int32_t>(
        (motor_position_deg * MotorConstants::ENCODER_STEPS) / MotorConstants::DEGREES_PER_REVOLUTION
    );

    RCLCPP_INFO(node_->get_logger(), "Setting joint %s position to %.2f radians (%.2f degrees on motor protactor) with gear ratio %.2f:1",
                joint_name.c_str(), position, motor_position_deg, joint.gear_ratio);
    RCLCPP_INFO(node_->get_logger(), "Calculated encoder counts: %d", encoder_counts);

    // Prepare CAN command
    uint16_t speed = 50;  // Default speed
    uint8_t acc_value = static_cast<uint8_t>(std::clamp(acceleration, 0.0, 255.0));

    std::vector<uint8_t> data = {
        CANCommands::ABSOLUTE_POSITION,  // 0xF5
        static_cast<uint8_t>((speed >> 8) & 0xFF),  // Speed high byte
        static_cast<uint8_t>(speed & 0xFF),         // Speed low byte
        acc_value,                                  // Acceleration
        static_cast<uint8_t>((encoder_counts >> 16) & 0xFF),  // Position high byte
        static_cast<uint8_t>((encoder_counts >> 8) & 0xFF),   // Position middle byte
        static_cast<uint8_t>(encoder_counts & 0xFF)           // Position low byte
    };

    // Send CAN command
    can_protocol_->sendFrame(joint.motor_id, data);

    // Store the commanded position
    joint.command_position = position;
    joint.last_command = node_->get_clock()->now();
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

    // Check velocity limits
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
    switch (static_cast<MotorMode>(joint.params.working_mode)) {
        case MotorMode::CR_OPEN:
        case MotorMode::SR_OPEN:
            max_speed = MotorConstants::MAX_RPM_OPEN;
            break;
        case MotorMode::CR_CLOSE:
        case MotorMode::SR_CLOSE:
            max_speed = MotorConstants::MAX_RPM_CLOSE;
            break;
        default:
            max_speed = MotorConstants::MAX_RPM_vFOC;
    }
    
    // Clamp speed to mode-specific max
    rpm = std::min(rpm, max_speed);
    
    // Prepare velocity command
    uint8_t direction = velocity >= 0 ? 0x00 : 0x80;
    uint16_t speed = static_cast<uint16_t>(rpm);
    
    // Format CAN message according to manual
    std::vector<uint8_t> data = {
        CANCommands::SPEED_CONTROL,
        static_cast<uint8_t>(direction | ((speed >> 8) & 0x0F)),  // Direction + high nibble of speed
        static_cast<uint8_t>(speed & 0xFF),  // Low byte of speed
        0x02  // Default acceleration
    };
    
    can_protocol_->sendFrame(joint.motor_id, data);
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
double MotorDriver::getJointPosition(const std::string& joint_name) const {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return 0.0;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Retrieved position for joint %s: %.3f", joint_name.c_str(), it->second.position);
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
 * Enables the motor for the specified joint.
 *
 * @param joint_name The name of the joint.
 */
void MotorDriver::enableMotor(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    std::vector<uint8_t> data = {CANCommands::ENABLE_MOTOR, 0x01};
    can_protocol_->sendFrame(it->second.motor_id, data);
    it->second.status.is_enabled = true;
}

/**
 * Disables the motor for the specified joint.
 *
 * @param joint_name The name of the joint.
 */
void MotorDriver::disableMotor(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    std::vector<uint8_t> data = {CANCommands::ENABLE_MOTOR, 0x00};
    can_protocol_->sendFrame(it->second.motor_id, data);
    it->second.status.is_enabled = false;
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
    std::vector<uint8_t> data = {CANCommands::EMERGENCY_STOP};
    can_protocol_->sendFrame(it->second.motor_id, data);
    
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
 * Sets the working mode of a motor.
 *
 * @param joint_name The name of the joint.
 * @param mode The desired working mode.
 */
void MotorDriver::setWorkingMode(const std::string& joint_name, MotorMode mode) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    std::vector<uint8_t> data = {
        CANCommands::SET_WORKING_MODE,
        static_cast<uint8_t>(mode)
    };
    can_protocol_->sendFrame(it->second.motor_id, data);
    it->second.params.working_mode = static_cast<uint8_t>(mode);
}

/**
 * Sets the working current for a specific joint.
 * 
 * @param joint_name The name of the joint.
 * @param current_ma The working current in milliamperes.
 *
 * TODO: Add failsafe for current limits
 */
void MotorDriver::setWorkingCurrent(const std::string& joint_name, uint16_t current_ma) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    std::vector<uint8_t> data = {
        CANCommands::SET_CURRENT,
        static_cast<uint8_t>(current_ma & 0xFF),
        static_cast<uint8_t>((current_ma >> 8) & 0xFF)
    };
    can_protocol_->sendFrame(it->second.motor_id, data);
    it->second.params.working_current = current_ma;
}

/**
 * Sets the holding current for a specific joint.
 * 
 * @param joint_name The name of the joint.
 * @param percentage The percentage of the holding current to set (between 10 and 90).
 * TODO: Add failsafe for current limits
 */
void MotorDriver::setHoldingCurrent(const std::string& joint_name, uint8_t percentage) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    percentage = std::clamp(percentage, uint8_t(10), uint8_t(90));
    uint8_t holding_value = (percentage - 10) / 10;  // Convert to 0-8 range

    std::vector<uint8_t> data = {0x9B, holding_value};
    can_protocol_->sendFrame(it->second.motor_id, data);
    it->second.params.holding_current_percentage = percentage;
}

/**
 * Calibrates the motor for the specified joint.
 *
 * @param joint_name The name of the joint to calibrate the motor for.
 * 
 * NOTE: Calibration should be done before tensioning the belt or adding any mechanical load.
 */
void MotorDriver::calibrateMotor(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Joint %s not found", joint_name.c_str());
        return;
    }

    std::vector<uint8_t> data = {CANCommands::CALIBRATE, 0x00};
    can_protocol_->sendFrame(it->second.motor_id, data);
    
    it->second.status.is_calibrated = false;  // Will be set true when calibration response received
    RCLCPP_INFO(node_->get_logger(), "Starting calibration for joint %s", joint_name.c_str());
}

/**
 * @brief Homes the motor for the specified joint.
 * 
 * This function homes the motor for the specified joint by sending a CAN frame with the GO_HOME command.
 * If the joint is not found, an error message is logged and the function returns.
 * 
 * @param joint_name The name of the joint to home the motor for.
 */
void MotorDriver::homeMotor(const std::string& joint_name) {
    auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "[homeMotor] Joint %s not found", joint_name.c_str());
        return;
    }

    auto& joint = it->second;
    auto elapsed_time = node_->get_clock()->now() - joint.last_update;
    if (elapsed_time.seconds() > 0.5) {
        RCLCPP_WARN(node_->get_logger(), "[homeMotor] Joint %s has stale state data (last update %.2f seconds ago), homing might fail",
                    joint_name.c_str(), elapsed_time.seconds());
    }

    std::vector<uint8_t> data = {CANCommands::GO_HOME};
    can_protocol_->sendFrame(it->second.motor_id, data);
    
    it->second.status.is_homed = false;  // Will be set true when homing complete
    RCLCPP_INFO(node_->get_logger(), "[homeMotor] Starting homing for joint %s", joint_name.c_str());
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
// void MotorDriver::updateJointStates() {
//     RCLCPP_INFO(node_->get_logger(), "updateJointStates is running...");
// }
void MotorDriver::updateJointStates() {
    static rclcpp::Time last_update = node_->now();
    auto now = node_->now();
    
    // Only update every 100ms
    // if ((now - last_update).seconds() < 0.5) {
    //     return;
    // }
    
    RCLCPP_INFO(node_->get_logger(), "[updateJointStates] Starting joint state update cycle");

    if (joints_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[updateJointStates] No joints registered for state updates");
        return;
    }

    for (auto& joint_pair : joints_) {
        auto& joint = joint_pair.second;

        // Skip updates while homing
        // NOTE: This will cause the issue "Not updating joint states when requires_homing is set to false and more than 1 joint configured" to occur

        // if (joint.status.is_homed == false) {
        //     RCLCPP_DEBUG(node_->get_logger(),
        //                  "[updateJointStates] Skipping updates for joint %s (homing in progress)",
        //                  joint.joint_name.c_str());
        //     continue;
        // }

        try {
            requestMotorData(joint.motor_id);
            // Request encoder position
            // std::vector<uint8_t> pos_request = {CANCommands::READ_ENCODER};
            // can_protocol_->sendFrame(joint.motor_id, pos_request);

            // // Request velocity
            // std::vector<uint8_t> vel_request = {CANCommands::READ_VELOCITY};
            // can_protocol_->sendFrame(joint.motor_id, vel_request);

            // // Request IO status
            // std::vector<uint8_t> io_request = {CANCommands::READ_IO};
            // can_protocol_->sendFrame(joint.motor_id, io_request);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[updateJointStates] Error requesting data for joint %s: %s",
                         joint.joint_name.c_str(), e.what());
        }
    }

    last_update = now;

    RCLCPP_INFO(node_->get_logger(), "[updateJointStates] Completed joint state update cycle");
}

void MotorDriver::processCANMessage(const can_msgs::msg::Frame::SharedPtr msg) {
    canMessageCallback(msg);
}

/**
 * @brief Callback function for CAN messages.
 * 
 * This function is called when a CAN message is received. It processes the message based on the command type and the motor ID.
 * 
 * @param msg The CAN message received.
 */
void MotorDriver::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg) {
    RCLCPP_ERROR(node_->get_logger(), "SUPER EARLY DEBUG - Raw CAN msg - ID: %d, DLC: %d, Data[0]: 0x%02X, Data[1]: 0x%02X", 
                 msg->id, msg->dlc, msg->data[0], msg->data[1]);
    RCLCPP_INFO(node_->get_logger(), "CAN message received - ID: %d, Command: 0x%02X", 
                msg->id, msg->data[0]);
    auto motor_it = motor_to_joint_map_.find(msg->id);
    if (motor_it == motor_to_joint_map_.end()) {
        RCLCPP_DEBUG(node_->get_logger(), "Received message for unknown motor ID: %d", msg->id);
        return;
    }

    std::vector<uint8_t> data(msg->data.begin(), msg->data.end());
    
    if (data.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received empty CAN message");
        return;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Received CAN message - ID: %d, Command: 0x%02X", 
                msg->id, data[0]);

    switch(data[0]) {
        case CANCommands::READ_ENCODER:
            RCLCPP_INFO(node_->get_logger(), "Processing encoder response");
            processEncoderResponse(msg->id, data);
            break;
            
        case CANCommands::READ_VELOCITY:
            RCLCPP_INFO(node_->get_logger(), "Processing velocity response");
            processVelocityResponse(msg->id, data);
            break;
            
        case CANCommands::READ_IO:
            RCLCPP_INFO(node_->get_logger(), "Processing IO response");
            processIOResponse(msg->id, data);
            break;
            
        case CANCommands::ABSOLUTE_POSITION:
        case CANCommands::CALIBRATE:
        case CANCommands::GO_HOME:
        case CANCommands::ENABLE_MOTOR:
            RCLCPP_INFO(node_->get_logger(), "Processing status response");
            processStatusResponse(msg->id, data);
            break;
            
        default:
            RCLCPP_INFO(node_->get_logger(), "Received unhandled command: 0x%02X", data[0]);
            break;
    }
}

/**
 * @brief Process the response from the encoder for a specific motor.
 *
 * This function extracts the encoder data from the received data and updates the joint state accordingly.
 * It calculates the position error by subtracting the current joint position from the commanded position.
 *
 * @param motor_id The ID of the motor.
 * @param data The received data from the encoder.
 */
void MotorDriver::processEncoderResponse(uint8_t motor_id, const std::vector<uint8_t>& data) {
    if (data.size() < 7) {
        RCLCPP_WARN(node_->get_logger(), "Encoder response too short: %zu bytes", data.size());
        return;
    }

    auto it = motor_to_joint_map_.find(motor_id);
    if (it == motor_to_joint_map_.end()) {
        RCLCPP_WARN(node_->get_logger(), "No joint found for motor ID: %d", motor_id);
        return;
    }

    const std::string& joint_name = it->second;
    auto& joint = joints_[joint_name];

    // Extract encoder data (bytes 1-6)
    std::vector<uint8_t> encoder_data(data.begin() + 1, data.begin() + 7);

    try {
        // Decode motor angle in degrees using decodeInt48
        double motor_angle_deg = CANProtocol::decodeInt48(encoder_data);

        // Sanity check for absurd values
        constexpr double MAX_MOTOR_DEGREES = 360.0 * 1000;  // Example limit: 1000 revolutions
        if (motor_angle_deg > MAX_MOTOR_DEGREES || motor_angle_deg < -MAX_MOTOR_DEGREES) {
            RCLCPP_WARN(node_->get_logger(), "Discarding out-of-range motor angle: %.2f degrees for motor ID %d", motor_angle_deg, motor_id);
            return;
        }

        // Convert motor angle to joint angle
        double joint_angle_deg = motor_angle_deg / joint.gear_ratio;
        double joint_angle_rad = joint_angle_deg * MotorConstants::DEG_TO_RAD;

        RCLCPP_INFO(node_->get_logger(), "Motor angle: %.2f degrees", motor_angle_deg);
        RCLCPP_INFO(node_->get_logger(), "Joint angle: %.2f radians", joint_angle_rad);

        // Update joint state
        joint.position = joint_angle_rad;
        joint.position_error = joint.command_position - joint.position;

        RCLCPP_INFO(node_->get_logger(), "Updated joint %s position: %.2f rad", 
                    joint_name.c_str(), joint.position);

        joint.last_update = node_->get_clock()->now();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error processing encoder response: %s", e.what());
    }
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
        double rpm = CANProtocol::decodeVelocityToRPM(velocity_data);
        // Use exact conversion factor
        joint.velocity = rpm * MotorConstants::RPM_TO_RADPS;
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

    // Decode limit switch states based on hardware type
    // TODO: Fix the logic
    if (joint.hardware_type == "MKS_42D") {
        // Remapped ports for MKS 42D
        joint.status.limit_switch_left = (io_status & (1 << 2)) != 0;  // IN_2 (Bit 2, Dir)
        joint.status.limit_switch_right = (io_status & (1 << 3)) != 0; // IN_1 (Bit 3, En)
    } else if (joint.hardware_type == "MKS_57D") {
        // Default ports for MKS 57D
        joint.status.limit_switch_left = (io_status & (1 << 3)) != 0;  // IN_1 (Bit 3)
        joint.status.limit_switch_right = (io_status & (1 << 2)) != 0; // IN_2 (Bit 2)
    } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown hardware type for motor ID: %d", motor_id);
        return;
    }

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
    RCLCPP_ERROR(node_->get_logger(), "EARLY DEBUG - Got status msg with cmd: 0x%02X status: 0x%02X", data[0], data[1]);
    RCLCPP_ERROR(node_->get_logger(), "processStatusResponse - data size: %zu, data[0]: %d (0x%02X), data[1]: %d (0x%02X)", 
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
    
    // Log raw data for debugging
    std::stringstream ss;
    ss << "Raw data: ";
    for (size_t i = 0; i < data.size(); i++) {
        ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0') 
           << static_cast<int>(data[i]) << " ";
    }
    RCLCPP_DEBUG(node_->get_logger(), "Status response for joint %s: %s",
                 joint.joint_name.c_str(), ss.str().c_str());

    switch(cmd) {
        case CANCommands::GO_HOME:
            RCLCPP_ERROR(node_->get_logger(), "Inside GO_HOME case - About to switch on status: %d (0x%02X)", 
                        data[1], data[1]);
            
            // Check for timeout
            if (!joint.status.is_homed) {
                auto elapsed_time = node_->get_clock()->now() - joint.last_command;
                if (elapsed_time.seconds() > 30.0) {
                    joint.status.is_error = true;
                    joint.status.error_message = "Homing timeout after 30 seconds";
                    RCLCPP_ERROR(node_->get_logger(), "Homing timeout for joint %s", joint.joint_name.c_str());
                    return;
                }
            }
            
            switch(status) {
                case 0:  // Homing failed
                    joint.status.is_homed = false;
                    joint.status.is_error = true;
                    joint.status.error_message = "Homing failed";
                    RCLCPP_DEBUG(node_->get_logger(), "Status 0: Homing failed");
                    break;
                    
                case 1:  // Homing started
                    joint.status.is_homed = false;
                    joint.status.is_error = false;
                    joint.status.error_message.clear();
                    RCLCPP_DEBUG(node_->get_logger(), "Status 1: Homing in progress");
                    break;
                    
                case 2:  // Homing successful
                    RCLCPP_DEBUG(node_->get_logger(), "Status 2: Homing complete");
                    joint.status.is_homed = true;
                    joint.status.is_error = false;
                    joint.status.error_message.clear();
                    // joint.position = 0.0;
                    updateJointStates();
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
            RCLCPP_ERROR(node_->get_logger(), "Inside ABSOLUTE_POSITION case - About to switch on status: %d (0x%02X)", 
                        data[1], data[1]);
            
            switch(status) {
                // TODO: Prevent from processing commands that aren't responses
                // case 0:  // Movement failed
                //     joint.status.is_error = true;
                //     joint.status.error_message = "Absolute position command failed";
                //     RCLCPP_DEBUG(node_->get_logger(), "Status 0: Failed to move to absolute position");
                //     break;
                    
                case 1:  // Movement started
                    joint.status.is_error = false;
                    joint.status.error_message.clear();
                    RCLCPP_DEBUG(node_->get_logger(), "Status 1: Move to absolute position started");
                    break;
                    
                case 2:  // Movement completed
                    RCLCPP_DEBUG(node_->get_logger(), "Status 2: Move to absolute position completed");
                    updateJointStates();
                    break;
                    
                case 3:  // Movement partially completed
                    /* 
                    TODO: Validate if this is necessary as the drivers have way of detecting if endstops are triggered.
                    
                    NOTE: We must verify this is true for a all joints

                    Based on Y:
                    1. If the position is a positive value, the motor is moving towards the left endstop
                        - joint.status.limit_switch_left = true
                    2. If the position is a negative value, the motor is moving towards the right endstop
                        - joint.status.limit_switch_right = true
                    
                    */
                    updateJointStates();
                    RCLCPP_DEBUG(node_->get_logger(), "Status 3: Move to absolute position partially completed (Endstop limit reached)");
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

    if (vel_max < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Invalid velocity limit for joint %s: %.2f",
                    joint_name.c_str(), vel_max);
        return;
    }

    if (acc_max < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Invalid acceleration limit for joint %s: %.2f",
                    joint_name.c_str(), acc_max);
        return;
    }

    // Update joint limits
    it->second.position_min = pos_min;
    it->second.position_max = pos_max;
    it->second.velocity_max = vel_max;
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
    // Request sequence of motor data
    std::vector<std::vector<uint8_t>> requests = {
        {CANCommands::READ_ENCODER},    // Position
        {CANCommands::READ_VELOCITY},   // Velocity
        {CANCommands::READ_IO},         // IO Status
        // {CANCommands::READ_ERROR}       // Error Status
    };

    for (const auto& request : requests) {
        can_protocol_->sendFrame(motor_id, request);
        // Add small delay between requests to prevent flooding
        std::this_thread::sleep_for(std::chrono::microseconds(100));
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