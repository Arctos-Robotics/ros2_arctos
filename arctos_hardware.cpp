#include "arctos_hardware_interface/arctos_hardware.hpp"
#include "motor_driver.hpp"
#include <string>
#include <vector>
#include <iostream>

namespace arctos_hardware_interface
{

CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(6, 0);
  joint_velocities_.assign(6, 0);
  joint_position_command_.assign(6, 0);
  joint_velocities_command_.assign(6, 0);

  // force sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  {
      // Loop through all motor IDs (assumed you have them in a list or array)
      for (size_t i = 0; i < joint_position_.size(); ++i)
      {
          uint32_t motor_id = joint_position_[i];

          // Get motor position using the motor driver's function
          double motor_position = motor_driver_.getMotorPosition(motor_id);

          // Store the motor position in the joint_positions array
          joint_position_[i] = motor_position;  // Assuming motor_ids_ and joint_positions_ are aligned

          // Optionally log for debugging
          RCLCPP_INFO(rclcpp::get_logger("robot_system"), "Motor %d Position: %f", motor_id, motor_position);
      }

      return hardware_interface::return_type::OK;
  }
  return return_type::OK;
}

return_type RobotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_position_command_.size(); ++i) {
      if (joint_position_command_[i] != joint_position_[i]) {
          RCLCPP_INFO(
              rclcpp::get_logger("RobotSystem"),
              "Received position command for joint %zu: %f",
              i,
              joint_position_command_[i]
          );
          // Simulate moving the joint towards the commanded position.
          joint_position_[i] = joint_position_command_[i];
      }
  }
  // Simulate additional state updates for the robot
  RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Command successfully applied.");
  return return_type::OK;
}

}  // namespace arctos_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arctos_hardware_interface::RobotSystem, hardware_interface::SystemInterface)
