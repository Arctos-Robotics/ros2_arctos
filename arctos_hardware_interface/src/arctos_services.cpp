#include "arctos_hardware_interface/arctos_services.hpp"
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

namespace arctos_interface {
namespace arctos_services {

Services::Services(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<arctos_motor_driver::MotorDriver> motor_driver,
    const std::vector<std::string>& joint_names,
    const std::vector<uint8_t>& motor_ids)
: node_(node),
  motor_driver_(motor_driver),
  joint_names_(joint_names),
  motor_ids_(motor_ids)
{
  RCLCPP_INFO(node_->get_logger(), "Creating services with %zu joints", joint_names.size());

  // Add test service to verify service functionality
  // TODO: REMOVE THIS SERVICE BEFORE DEPLOYMENT
  auto test_service = node_->create_service<std_srvs::srv::Trigger>(
      "test_service",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          RCLCPP_INFO(node_->get_logger(), "Test service called!");
          response->success = true;
          response->message = "Test service works!";
      });
  // Get node name for proper namespacing
  std::string ns = node_->get_name();
  
  // Create global services with explicit names
  calibrate_all_service_ = node_->create_service<std_srvs::srv::Trigger>(
    ns + "/calibrate_all_motors",  // Removed ~
    std::bind(&Services::handleCalibrationAllRequest, this,
              std::placeholders::_1, std::placeholders::_2));
              
  home_all_service_ = node_->create_service<std_srvs::srv::Trigger>(
    ns + "/home_all_motors",  // Removed ~
    std::bind(&Services::handleHomingAllRequest, this,
              std::placeholders::_1, std::placeholders::_2));

  estop_service_ = node_->create_service<std_srvs::srv::Trigger>(
    ns + "/emergency_stop",  // Removed ~
    std::bind(&Services::handleEmergencyStop, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create per-joint services with explicit names
  for (const auto& joint_name : joint_names_) {
    auto calib_service = node_->create_service<std_srvs::srv::Trigger>(
      ns + "/calibrate_joint/" + joint_name,  // Removed ~
      [this, name = joint_name](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          this->handleCalibrationJointRequest(name, req, res);
      });
    calibrate_joint_services_.push_back(calib_service);

    auto home_service = node_->create_service<std_srvs::srv::Trigger>(
      ns + "/home_joint/" + joint_name,  // Removed ~
      [this, name = joint_name](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          this->handleHomingJointRequest(name, req, res);
      });
    home_joint_services_.push_back(home_service);

    // Initialize status tracking
    calibration_in_progress_[joint_name] = false;
    homing_in_progress_[joint_name] = false;
  }

  // Create status publisher with explicit name
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    ns + "/status", 10);  // Removed ~

  // Create status timer
  status_timer_ = node_->create_wall_timer(
    100ms,  // Check status every 100ms
    std::bind(&Services::publishStatus, this));
}

void Services::handleCalibrationAllRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Starting calibration of all motors...");
  
  bool all_success = true;
  std::stringstream message;
  
  for (const auto& joint_name : joint_names_) {
    try {
      motor_driver_->calibrateMotor(joint_name);
      calibration_in_progress_[joint_name] = true;
      message << "Started calibration of joint " << joint_name << "\n";
    } catch (const std::exception& e) {
      all_success = false;
      message << "Failed to start calibration of joint " << joint_name 
              << ": " << e.what() << "\n";
      RCLCPP_ERROR(node_->get_logger(), "Calibration failed for joint %s: %s",
                   joint_name.c_str(), e.what());
    }
  }
  
  response->success = all_success;
  response->message = message.str();
}

void Services::handleHomingAllRequest(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Starting homing of all motors...");
  
  bool all_success = true;
  std::stringstream message;
  
  for (const auto& joint_name : joint_names_) {
    try {
      motor_driver_->homeMotor(joint_name);
      homing_in_progress_[joint_name] = true;
      message << "Started homing of joint " << joint_name << "\n";
    } catch (const std::exception& e) {
      all_success = false;
      message << "Failed to start homing of joint " << joint_name 
              << ": " << e.what() << "\n";
      RCLCPP_ERROR(node_->get_logger(), "Homing failed for joint %s: %s",
                   joint_name.c_str(), e.what());
    }
  }
  
  response->success = all_success;
  response->message = message.str();
}

void Services::handleCalibrationJointRequest(
  const std::string& joint_name,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Starting calibration of joint %s...", joint_name.c_str());
  
  try {
    motor_driver_->calibrateMotor(joint_name);
    calibration_in_progress_[joint_name] = true;
    response->success = true;
    response->message = "Started calibration of joint " + joint_name;
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to start calibration: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "Failed to start calibration of joint %s: %s",
                 joint_name.c_str(), e.what());
  }
}

void Services::handleHomingJointRequest(
  const std::string& joint_name,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "Received homing request for joint %s", joint_name.c_str());
  
  try {
    // Find motor ID for this joint
    auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
    if (it == joint_names_.end()) {
      throw std::runtime_error("Joint name not found");
    }
    size_t index = std::distance(joint_names_.begin(), it);
    uint8_t motor_id = motor_ids_[index];
    
    RCLCPP_INFO(node_->get_logger(), "Starting homing for joint %s (motor ID: %d)...", 
                joint_name.c_str(), motor_id);
    
    motor_driver_->homeMotor(joint_name);
    homing_in_progress_[joint_name] = true;
    
    response->success = true;
    response->message = "Started homing of joint " + joint_name;
    
    RCLCPP_INFO(node_->get_logger(), "Homing initiated successfully");
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to start homing: " + std::string(e.what());
    RCLCPP_ERROR(node_->get_logger(), "Failed to start homing of joint %s: %s",
                 joint_name.c_str(), e.what());
  }
}

void Services::handleEmergencyStop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_WARN(node_->get_logger(), "Emergency stop triggered!");
  
  bool all_success = true;
  std::stringstream message;
  
  // Stop all ongoing operations
  for (auto& pair : calibration_in_progress_) {
    pair.second = false;
  }
  for (auto& pair : homing_in_progress_) {
    pair.second = false;
  }
  
  // Stop all motors
  try {
    motor_driver_->stopAllMotors();
    message << "Successfully stopped all motors\n";
  } catch (const std::exception& e) {
    all_success = false;
    message << "Error stopping motors: " << e.what() << "\n";
  }
  
  response->success = all_success;
  response->message = message.str();
}

void Services::publishStatus()
{
  auto msg = std::make_unique<std_msgs::msg::String>();
  std::stringstream ss;
  
  // Check status of all joints
  for (const auto& joint_name : joint_names_) {
    try {
      auto status = motor_driver_->getMotorStatus(joint_name);
      ss << "Joint " << joint_name << ":\n";
      ss << "  Enabled: " << (status.is_enabled ? "true" : "false") << "\n";
      ss << "  Calibrated: " << (status.is_calibrated ? "true" : "false") << "\n";
      ss << "  Homed: " << (status.is_homed ? "true" : "false") << "\n";
      ss << "  Error: " << (status.is_error ? status.error_message : "none") << "\n";
      ss << "  Calibrating: " << (calibration_in_progress_[joint_name] ? "true" : "false") << "\n";
      ss << "  Homing: " << (homing_in_progress_[joint_name] ? "true" : "false") << "\n";
      
      if (calibration_in_progress_[joint_name]) {
        monitorCalibrationStatus(joint_name);
      }
      
      if (homing_in_progress_[joint_name]) {
        monitorHomingStatus(joint_name);
      }
      
    } catch (const std::exception& e) {
      ss << "Joint " << joint_name << ": Error getting status - " << e.what() << "\n";
    }
  }
  
  msg->data = ss.str();
  status_pub_->publish(std::move(msg));
}

void Services::monitorCalibrationStatus(const std::string& joint_name)
{
  try {
    auto status = motor_driver_->getMotorStatus(joint_name);
    if (status.is_calibrated) {
      calibration_in_progress_[joint_name] = false;
      RCLCPP_INFO(node_->get_logger(), "Calibration completed for joint %s", 
                  joint_name.c_str());
    } else if (status.is_error) {
      calibration_in_progress_[joint_name] = false;
      RCLCPP_ERROR(node_->get_logger(), "Calibration failed for joint %s: %s",
                  joint_name.c_str(), status.error_message.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error monitoring calibration status for joint %s: %s",
                joint_name.c_str(), e.what());
    calibration_in_progress_[joint_name] = false;
  }
}

void Services::monitorHomingStatus(const std::string& joint_name)
{
  try {
    auto status = motor_driver_->getMotorStatus(joint_name);
    if (status.is_homed) {
      homing_in_progress_[joint_name] = false;
      RCLCPP_INFO(node_->get_logger(), "Homing completed for joint %s", 
                  joint_name.c_str());
    } else if (status.is_error) {
      homing_in_progress_[joint_name] = false;
      RCLCPP_ERROR(node_->get_logger(), "Homing failed for joint %s: %s",
                  joint_name.c_str(), status.error_message.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Error monitoring homing status for joint %s: %s",
                joint_name.c_str(), e.what());
    homing_in_progress_[joint_name] = false;
  }
}

}  // namespace arctos_services
}  // namespace arctos_interface