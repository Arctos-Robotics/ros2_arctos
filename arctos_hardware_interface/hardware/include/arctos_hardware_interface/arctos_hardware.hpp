#ifndef ARCTOS_HARDWARE_INTERFACE_ARCTOS_HARDWARE_HPP_
#define ARCTOS_HARDWARE_INTERFACE_ARCTOS_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "motor_driver.hpp" 

using hardware_interface::return_type;

namespace arctos_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC RobotSystem : public hardware_interface::SystemInterface
{
public:
  
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  // Vectors to store joint states and commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  // MotorDriver object to interface with motors
  MotorDriver motor_driver_;  // Declare motor_driver_ as a member variable

  // Joint interfaces map (positions, velocities, etc.)
  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  // namespace arctos_hardware_interface

#endif  // ARCTOS_HARDWARE_INTERFACE_ARCTOS_HARDWARE_HPP_
