// include/arctos_hardware/single_joint_hardware.hpp
#ifndef MY_ROBOT_HARDWARE__SINGLE_JOINT_HARDWARE_HPP_
#define MY_ROBOT_HARDWARE__SINGLE_JOINT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "can_msgs/msg/frame.hpp"

namespace arctos_hardware
{

class SingleJointHardware : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SingleJointHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  uint32_t can_id_;
  std::string can_interface_;
  
  // State and command interfaces
  double hw_position_;
  double hw_velocity_;
  double hw_effort_;
  double hw_command_;
  
  // ROS2 communication
  std::shared_ptr<rclcpp::Node> can_node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  // CAN message handling
  void canCallback(const can_msgs::msg::Frame::SharedPtr msg);
  bool validateCanId(const can_msgs::msg::Frame::SharedPtr msg) const;
};

}  // namespace arctos_hardware

#endif