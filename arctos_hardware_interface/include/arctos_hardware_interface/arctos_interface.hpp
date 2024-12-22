#ifndef ARCTOS_HARDWARE_INTERFACE_HPP_
#define ARCTOS_HARDWARE_INTERFACE_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include "arctos_motor_driver/motor_driver.hpp"
#include "arctos_motor_driver/can_protocol.hpp"
#include "arctos_hardware_interface/arctos_services.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arctos_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::return_type;

class HARDWARE_INTERFACE_PUBLIC ArctosInterface : public hardware_interface::SystemInterface
{
public:
  ArctosInterface();
  ~ArctosInterface();
  
  // ROS 2 Control Interface
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Tracking of last commanded positions and velocities
  std::vector<double> last_position_command_;
  std::vector<double> last_velocity_command_;
  
  // Tolerance values for filtering commands
  double position_tolerance_ = 0.001;  // Default value, can be overridden or set via parameter
  double velocity_tolerance_ = 0.01;   // Default value, can be overridden or set via parameter

  // Joint state storage
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  // Interface mapping
  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, 
    {"velocity", {}}
  };

  // Motor driver components
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<arctos_motor_driver::MotorDriver> motor_driver_;
  std::shared_ptr<arctos_motor_driver::CANProtocol> can_protocol_;
  
  // Configuration parameters
  std::vector<uint8_t> motor_ids_;  // Mapping of joint indices to motor IDs
  bool has_velocity_interface_{false};
  bool has_position_interface_{false};

private:
  // Services implementation
  std::unique_ptr<arctos_services::Services> services_;
  
  // Helper functions for motor initialization
  void initializeMotors();
  bool setupMotorParameters(const hardware_interface::ComponentInfo& joint_info, uint8_t motor_id);
};

}  // namespace arctos_interface

#endif  // ARCTOS_HARDWARE_INTERFACE_HPP_