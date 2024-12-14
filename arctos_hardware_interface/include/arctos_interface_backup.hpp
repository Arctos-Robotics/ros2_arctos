#ifndef ARCTOS_INTERFACE_HPP_
#define ARCTOS_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "motor_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>
#include <memory>
#include "pluginlib/class_list_macros.hpp"
namespace arctos_interface
{
  class ArctosInterface : public hardware_interface::SystemInterface
  {
  public:
    ArctosInterface();

    // Init function
    void init();

    // Reset function
    void reset();

    // Read function: Read current joint positions from the hardware
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);

    // Write function: Send joint position commands to the hardware
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

    // Export command interfaces using CommandInterface
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();
    std::vector<hardware_interface::StateInterface> export_state_interfaces();
  private:
    std::shared_ptr<MotorDriver> motor_driver_;  // MotorDriver instance for communication with hardware
    std::vector<double> joint_position_;  // Current joint positions
    std::vector<double> joint_position_command_;  // Commanded joint positions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;  // Subscriber to joint state messages

    // Update joint positions from the received message
    void updateJointPositions(const sensor_msgs::msg::JointState::SharedPtr msg);
  };
}  // namespace arctos_interface

#endif  // ARCTOS_INTERFACE_HPP_
