#include "arctos_hardware_interface/arctos_interface.hpp"
// #include "arctos_hardware_interface/arctos_services.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <vector>
#include <chrono>

using hardware_interface::return_type;
using hardware_interface::CallbackReturn;
using arctos_motor_driver::MotorMode;

namespace arctos_interface
{

ArctosInterface::ArctosInterface()
: SystemInterface(),
  node_(std::make_shared<rclcpp::Node>("arctos_hardware_interface"))
{
  can_protocol_ = std::make_shared<arctos_motor_driver::CANProtocol>(node_);
  motor_driver_ = std::make_shared<arctos_motor_driver::MotorDriver>(node_);
  motor_driver_->setCAN(can_protocol_);

  // Add periodic status check
  node_->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      RCLCPP_DEBUG(node_->get_logger(), "Node is alive and spinning");
    });
}

ArctosInterface::~ArctosInterface() = default;

CallbackReturn ArctosInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize state storage vectors
  joint_position_.resize(info_.joints.size(), 0.0);
  joint_velocities_.resize(info_.joints.size(), 0.0);
  joint_position_command_.resize(info_.joints.size(), 0.0);
  joint_velocities_command_.resize(info_.joints.size(), 0.0);
  motor_ids_.resize(info_.joints.size());

  // Force/torque sensor has 6 readings
  ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);

  // Clear previous interface mappings
  joint_interfaces["position"].clear();
  joint_interfaces["velocity"].clear();

  // Collect joint names for service initialization
  std::vector<std::string> joint_names;
  joint_names.reserve(info_.joints.size());

  // Process joints and their interfaces
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints[i];
    joint_names.push_back(joint.name);
    
    // Create parameter name for this joint's settings
    std::string param_prefix = "motors." + joint.name + ".";
    
    // Declare parameters for this joint
    node_->declare_parameter(param_prefix + "motor_id", -1);         // Motor/CAN ID
    // node_->declare_parameter(param_prefix + "working_current", 1600); // Default 1.6A
    // node_->declare_parameter(param_prefix + "holding_current", 50);   // Default 50%
    // node_->declare_parameter(param_prefix + "home_current", 800);     // Default 0.8A for homing
    node_->declare_parameter(param_prefix + "requires_homing", false);// Default no homing needed
    
    // Get motor ID from parameters
    int motor_id;
    if (!node_->get_parameter(param_prefix + "motor_id", motor_id)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get motor_id for joint %s", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    
    if (motor_id < 0) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid motor_id (%d) for joint %s", motor_id, joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    motor_ids_[i] = static_cast<uint8_t>(motor_id);
    
    RCLCPP_INFO(node_->get_logger(), "Configured joint %s with motor_id %d", 
                joint.name.c_str(), motor_id);

    // Track available interfaces
    for (const auto & interface : joint.state_interfaces) {
      joint_interfaces[interface.name].push_back(joint.name);
      if (interface.name == "position") has_position_interface_ = true;
      if (interface.name == "velocity") has_velocity_interface_ = true;
    }
  }

  // Initialize services
  // services_ = std::make_unique<arctos_services::Services>(
  //   node_, 
  //   motor_driver_, 
  //   joint_names,
  //   motor_ids_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArctosInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  auto can_sub = node_->create_subscription<can_msgs::msg::Frame>(
      "/from_motor_can_bus", 
      rclcpp::QoS(10).reliable(),
      [this](const can_msgs::msg::Frame::SharedPtr msg) {
          RCLCPP_ERROR(node_->get_logger(), 
                      "CAN Message Received - Topic: %s, ID: %d, Data Length: %zu", 
                      "/from_motor_can_bus", msg->id, msg->data.size());
          
          // Log raw message data
          std::stringstream data_str;
          for (size_t i = 0; i < msg->data.size(); ++i) {
              data_str << std::hex << static_cast<int>(msg->data[i]) << " ";
          }
          RCLCPP_ERROR(node_->get_logger(), "Message Data: %s", data_str.str().c_str());

          // Call processCANMessage
          motor_driver_->processCANMessage(msg);
      }
  );
    try {
        initializeMotors();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize motors: %s", e.what());
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArctosInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(node_->get_logger(), "Transitioning to ACTIVE state from %s", previous_state.label().c_str());
  // Enable all motors
  for (size_t i = 0; i < info_.joints.size(); i++) {
    try {
      const auto& joint_name = info_.joints[i].name;
      
      RCLCPP_INFO(node_->get_logger(), "Enabling motor for joint %s", joint_name.c_str());
      // Enable the motor first
      motor_driver_->enableMotor(joint_name);
      
      // Check if homing is required
      bool requires_homing = false;
      std::string param_prefix = "motors." + joint_name + ".";
      if (node_->get_parameter(param_prefix + "requires_homing", requires_homing) && requires_homing) {
        RCLCPP_INFO(node_->get_logger(), "Starting homing sequence for joint %s", joint_name.c_str());
        
        // Get homing current if specified
        // int home_current = 800;  // Default 0.8A
        // node_->get_parameter(param_prefix + "home_current", home_current);
        
        // Configure motor for homing
        // auto current_status = motor_driver_->getMotorStatus(joint_name);
        // uint16_t original_current = current_status.params.working_current;
        // motor_driver_->setWorkingCurrent(joint_name, static_cast<uint16_t>(home_current));
        
        // Start homing
        motor_driver_->homeMotor(joint_name);
        
        // Wait for homing to complete (with timeout)
        rclcpp::Time start_time = node_->now();
        while (!motor_driver_->getMotorStatus(joint_name).is_homed) {
          if ((node_->now() - start_time).seconds() > 30.0) {  // 30 second timeout
            RCLCPP_ERROR(node_->get_logger(), "Homing timeout for joint %s", joint_name.c_str());
            return CallbackReturn::ERROR;
          }
          if (motor_driver_->getMotorStatus(joint_name).is_error) {
            RCLCPP_ERROR(node_->get_logger(), "Homing error for joint %s", joint_name.c_str());
            return CallbackReturn::ERROR;
          }
          rclcpp::spin_some(node_);
          rclcpp::sleep_for(std::chrono::milliseconds(100));  // Check every 100ms
        }
        
        // Restore original current
        // motor_driver_->setWorkingCurrent(joint_name, original_current);
        RCLCPP_INFO(node_->get_logger(), "Homing completed for joint %s", joint_name.c_str());
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to activate joint %s: %s",
                   info_.joints[i].name.c_str(), e.what());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArctosInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(node_->get_logger(), "Transitioning to INACTIVE state from %s", previous_state.label().c_str());
  // Disable all motors
  for (size_t i = 0; i < info_.joints.size(); i++) {
    try {
      motor_driver_->disableMotor(info_.joints[i].name);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to disable motor for joint %s: %s",
                   info_.joints[i].name.c_str(), e.what());
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArctosInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (has_position_interface_) {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]);
    }
    if (has_velocity_interface_) {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
    }
  }

  // Add force/torque sensor interfaces
  state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArctosInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Add joint command interfaces
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (has_position_interface_) {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]);
    }
    if (has_velocity_interface_) {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_command_[i]);
    }
  }

  // Add force/torque sensor command interfaces
  command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

  return command_interfaces;
}

return_type ArctosInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    static rclcpp::Time last_request = time;
    
    // Request updates at 50Hz
    if ((time - last_request).seconds() >= 0.02) {
        for (size_t i = 0; i < info_.joints.size(); i++) {
            const std::string& joint_name = info_.joints[i].name;
            // Request both position and velocity simultaneously
            motor_driver_->requestJointUpdate(joint_name, true, true);
            
            // Update joint states from motor driver's cached values
            if (has_position_interface_) {
                joint_position_[i] = motor_driver_->getJointPosition(joint_name);
            }
            if (has_velocity_interface_) {
                joint_velocities_[i] = motor_driver_->getJointVelocity(joint_name);
            }
        }
        last_request = time;
    }

    return return_type::OK;
}

return_type ArctosInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Resize last command vectors if not already done
  if (last_position_command_.size() != info_.joints.size()) {
    last_position_command_.resize(info_.joints.size(), 0.0);
    last_velocity_command_.resize(info_.joints.size(), 0.0);
  }

  for (size_t i = 0; i < info_.joints.size(); i++) {
    try {
      if (has_position_interface_) {
        // Only send if position has changed significantly
        if (std::abs(joint_position_command_[i] - last_position_command_[i]) > position_tolerance_) {
          motor_driver_->setJointPosition(info_.joints[i].name, joint_position_command_[i]);
          last_position_command_[i] = joint_position_command_[i];
        }
      }
      
      if (has_velocity_interface_) {
        // Only send if velocity has changed significantly
        if (std::abs(joint_velocities_command_[i] - last_velocity_command_[i]) > velocity_tolerance_) {
          motor_driver_->setJointVelocity(info_.joints[i].name, joint_velocities_command_[i]);
          last_velocity_command_[i] = joint_velocities_command_[i];
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to write command to joint %s: %s",
                   info_.joints[i].name.c_str(), e.what());
      return return_type::ERROR;
    }
  }

  return return_type::OK;
}

void ArctosInterface::initializeMotors() {
  for (size_t i = 0; i < info_.joints.size(); i++) {
      const auto& joint = info_.joints[i];
      uint8_t motor_id = motor_ids_[i];

      // Get gear ratio parameter
      std::string param_prefix = "motors." + joint.name + ".";
      node_->declare_parameter(param_prefix + "gear_ratio", 1.0);
      
      double gear_ratio;
      if (!node_->get_parameter(param_prefix + "gear_ratio", gear_ratio)) {
          RCLCPP_WARN(node_->get_logger(), "No gear ratio specified for joint %s, using 1:1", 
                      joint.name.c_str());
          gear_ratio = 1.0;
      }

      // Add joint to motor driver with gear ratio
      motor_driver_->addJoint(joint.name, motor_id, gear_ratio);

      // Configure motor parameters
      if (!setupMotorParameters(joint, motor_id)) {
          throw std::runtime_error("Failed to configure motor for joint " + joint.name);
      }

      RCLCPP_INFO(node_->get_logger(), "Initialized motor for joint %s with ID %d and gear ratio %.2f:1",
                  joint.name.c_str(), motor_id, gear_ratio);
  }
}

bool ArctosInterface::setupMotorParameters(
  const hardware_interface::ComponentInfo& joint_info, uint8_t motor_id)
{
  try {
    // Set working mode (default to SR_vFOC)
    motor_driver_->setWorkingMode(joint_info.name, MotorMode::SR_vFOC);

    // Get working current from parameters
    // int working_current;
    // std::string param_prefix = "motors." + joint_info.name + ".";
    // if (node_->get_parameter(param_prefix + "working_current", working_current)) {
    //   motor_driver_->setWorkingCurrent(joint_info.name, static_cast<uint16_t>(working_current));
    // }

    // // Get holding current from parameters
    // int holding_current;
    // if (node_->get_parameter(param_prefix + "holding_current", holding_current)) {
    //   motor_driver_->setHoldingCurrent(joint_info.name, static_cast<uint8_t>(holding_current));
    // }

    // Set position limits if specified
    auto pos_min_param = joint_info.parameters.find("position_min");
    auto pos_max_param = joint_info.parameters.find("position_max");
    auto vel_max_param = joint_info.parameters.find("velocity_max");
    auto acc_max_param = joint_info.parameters.find("acceleration_max");

    if (pos_min_param != joint_info.parameters.end() &&
        pos_max_param != joint_info.parameters.end() &&
        vel_max_param != joint_info.parameters.end() &&
        acc_max_param != joint_info.parameters.end()) 
    {
      double pos_min = std::stod(pos_min_param->second);
      double pos_max = std::stod(pos_max_param->second);
      double vel_max = std::stod(vel_max_param->second);
      double acc_max = std::stod(acc_max_param->second);
      
      // motor_driver_->setJointLimits(joint_info.name, pos_min, pos_max, vel_max, acc_max);
    }

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to setup motor parameters for joint %s: %s",
                 joint_info.name.c_str(), e.what());
    return false;
  }
}

}  // namespace arctos_interface

PLUGINLIB_EXPORT_CLASS(
  arctos_interface::ArctosInterface, 
  hardware_interface::SystemInterface)