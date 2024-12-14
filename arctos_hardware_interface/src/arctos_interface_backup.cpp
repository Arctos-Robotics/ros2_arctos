#include "arctos_interface.hpp"

namespace arctos_interface
{
  ArctosInterface::ArctosInterface()
    : motor_driver_(std::make_shared<MotorDriver>())
  {
    joint_position_.resize(6, 0.0);  // Assuming 6 motors (adjust as needed)
    joint_position_command_.resize(6, 0.0);  // Initialize command positions (adjust size as needed)

    // Subscribe to joint states published by the MotorDriver
    joint_state_sub_ = motor_driver_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->updateJointPositions(msg);  // Callback to update joint positions
      });
  }

  // Init function
  void ArctosInterface::init()
  {
    RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"), "ArctosInterface initialized.");
  }

  // Reset function
  void ArctosInterface::reset()
  {
    RCLCPP_INFO(rclcpp::get_logger("ArctosInterface"), "ArctosInterface reset.");
  }

  // Read function: Read current joint positions from the hardware
  hardware_interface::return_type ArctosInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    for (size_t i = 0; i < joint_position_.size(); ++i)
    {
      joint_position_[i] = motor_driver_->getMotorPosition(i);  // Retrieve position from the motor driver
    }
    return hardware_interface::return_type::OK;
  }

  // Write function: Send joint position commands to the hardware
  hardware_interface::return_type ArctosInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    for (uint32_t i = 0; i < joint_position_command_.size(); ++i)
    {
      int32_t target_position = static_cast<int32_t>(joint_position_command_[i] * 0x4000);  // Example scaling factor
      uint16_t speed = 600;  // Example speed (adjust as needed)
      uint8_t acceleration = 2;  // Example acceleration (adjust as needed)

      motor_driver_->sendPositionCommand(i, target_position, speed, acceleration);  // Send command to the motor
    }

    return hardware_interface::return_type::OK;
  }
  std::vector<hardware_interface::StateInterface> ArctosInterface::export_state_interfaces()
  {
      // Return a vector of state interfaces (if required by your hardware interface)
      std::vector<hardware_interface::StateInterface> state_interfaces;
      // Implement your state interface export logic here if needed
      return state_interfaces;
  }
  std::vector<hardware_interface::CommandInterface> ArctosInterface::export_command_interfaces()
  {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      for (size_t i = 0; i < joint_position_command_.size(); ++i)
      {
          // Ensure we're using a raw pointer to joint_position_command_[i]
          double* joint_position_ptr = &joint_position_command_[i];  // Raw pointer

          // Create CommandInterface with raw pointer to joint position
          command_interfaces.push_back(hardware_interface::CommandInterface(
              "joint_" + std::to_string(i + 1),    // Joint name
              "position",                          // Interface name (position)
              joint_position_ptr                   // Raw pointer to the joint position
          ));
      }

      return command_interfaces;
  }

  // Update joint positions from the received message
  void ArctosInterface::updateJointPositions(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->position.size(); ++i)
    {
      if (i < joint_position_.size()) {
        joint_position_[i] = msg->position[i];  // Update joint position
      }
    }
  }

}  // End of namespace arctos_interface

// This line should be outside the class and namespace scope.
PLUGINLIB_EXPORT_CLASS(arctos_interface::ArctosInterface, hardware_interface::SystemInterface)
