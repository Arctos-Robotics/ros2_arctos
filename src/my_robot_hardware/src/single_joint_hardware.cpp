#include "my_robot_hardware/single_joint_hardware.hpp"
#include <chrono>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn SingleJointHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage
  hw_position_ = 0.0;
  hw_velocity_ = 0.0;
  hw_effort_ = 0.0;
  hw_command_ = 0.0;

  // Get parameters
  can_interface_ = info_.hardware_parameters["can_interface"];
  if (can_interface_.empty()) {
    can_interface_ = "can0";
  }

  const auto& can_id_str = info_.hardware_parameters["can_id"];
  if (can_id_str.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("SingleJointHardware"), "No CAN ID specified");
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_id_ = std::stoul(can_id_str, nullptr, 16);

  // Setup ROS2 communication
  can_node_ = std::make_shared<rclcpp::Node>("can_hardware_interface_" + info_.joints[0].name);
  
  can_pub_ = can_node_->create_publisher<can_msgs::msg::Frame>(
    "/" + can_interface_ + "/transmit", 10);
  
  can_sub_ = can_node_->create_subscription<can_msgs::msg::Frame>(
    "/" + can_interface_ + "/receive", 10,
    std::bind(&SingleJointHardware::canCallback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

void SingleJointHardware::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (!validateCanId(msg)) {
    return;
  }

  // Process received CAN frame for this specific joint's MKS servo
  // Implement according to MKS servo protocol
}

bool SingleJointHardware::validateCanId(const can_msgs::msg::Frame::SharedPtr msg) const
{
  return msg->id == can_id_;
}

std::vector<hardware_interface::StateInterface> SingleJointHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_EFFORT, &hw_effort_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SingleJointHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_command_));

  return command_interfaces;
}

hardware_interface::CallbackReturn SingleJointHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SingleJointHardware"), 
              "Activating joint with CAN ID: 0x%x", can_id_);
  
  if (std::isnan(hw_position_)) {
    hw_position_ = 0.0;
  }
  if (std::isnan(hw_velocity_)) {
    hw_velocity_ = 0.0;
  }
  if (std::isnan(hw_effort_)) {
    hw_effort_ = 0.0;
  }
  hw_command_ = hw_position_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SingleJointHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SingleJointHardware"), 
              "Deactivating joint with CAN ID: 0x%x", can_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SingleJointHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(can_node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SingleJointHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send command to MKS servo
  can_msgs::msg::Frame frame;
  frame.id = can_id_;
  frame.dlc = 8;
  frame.is_rtr = false;
  frame.is_extended = false;
  
  // Fill frame data according to MKS servo protocol
  
  can_pub_->publish(frame);
  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::SingleJointHardware, hardware_interface::ActuatorInterface)