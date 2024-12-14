#include "my_robot_hardware/arm_system_hardware.hpp"
#include <chrono>

namespace my_robot_hardware
{

hardware_interface::CallbackReturn ArmSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  can_ids_.resize(info_.joints.size(), 0);

  can_interface_ = info_.hardware_parameters["can_interface"];
  if (can_interface_.empty()) {
    can_interface_ = "can0";
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints[i];
    auto id_it = joint.parameters.find("can_id");
    if (id_it == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("ArmSystemHardware"),
                   "No CAN ID specified for joint %s", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    can_ids_[i] = std::stoul(id_it->second, nullptr, 16);
  }

  can_node_ = std::make_shared<rclcpp::Node>("can_hardware_interface");
  
  can_pub_ = can_node_->create_publisher<can_msgs::msg::Frame>(
    "/" + can_interface_ + "/transmit", 10);
  
  can_sub_ = can_node_->create_subscription<can_msgs::msg::Frame>(
    "/" + can_interface_ + "/receive", 10,
    std::bind(&ArmSystemHardware::canCallback, this, std::placeholders::_1));

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ArmSystemHardware::canCallback(const can_msgs::msg::Frame::SharedPtr /*msg*/)
{
  // Process received CAN frames based on your protocol
}

std::vector<hardware_interface::StateInterface> ArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"), "Activating...");
  
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    if (std::isnan(hw_positions_[i])) {
      hw_positions_[i] = 0.0;
    }
    if (std::isnan(hw_velocities_[i])) {
      hw_velocities_[i] = 0.0;
    }
    if (std::isnan(hw_efforts_[i])) {
      hw_efforts_[i] = 0.0;
    }
    hw_commands_[i] = hw_positions_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArmSystemHardware"), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArmSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  rclcpp::spin_some(can_node_);
  
  static std::vector<double> last_positions = hw_positions_;
  static auto last_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  
  std::chrono::duration<double> dt = current_time - last_time;
  if (dt.count() > 0) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      hw_velocities_[i] = (hw_positions_[i] - last_positions[i]) / dt.count();
      last_positions[i] = hw_positions_[i];
    }
    last_time = current_time;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (size_t i = 0; i < info_.joints.size(); i++) {
    can_msgs::msg::Frame frame;
    frame.id = can_ids_[i];
    frame.dlc = 8;
    frame.is_rtr = false;
    frame.is_extended = false;
    
    // Fill frame data based on your protocol
    
    can_pub_->publish(frame);
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::ArmSystemHardware, hardware_interface::ActuatorInterface)