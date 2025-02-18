#ifndef ARCTOS_SERVICES_HPP_
#define ARCTOS_SERVICES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "arctos_motor_driver/motor_driver.hpp"
#include <map>
#include <string>
#include <memory>
#include <vector>

namespace arctos_interface {
namespace arctos_services {

struct Services {
  explicit Services(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<arctos_motor_driver::MotorDriver> motor_driver,
    const std::vector<std::string>& joint_names,
    const std::vector<uint8_t>& motor_ids);

  // Service handlers
  void handleCalibrationAllRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void handleHomingAllRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleCalibrationJointRequest(
    const std::string& joint_name,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
  void handleHomingJointRequest(
    const std::string& joint_name,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void handleEmergencyStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Status monitoring
  void publishStatus();
  void monitorCalibrationStatus(const std::string& joint_name);
  void monitorHomingStatus(const std::string& joint_name);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<arctos_motor_driver::MotorDriver> motor_driver_;
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> motor_ids_;
  
  // ROS 2 Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_all_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_all_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> calibrate_joint_services_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> home_joint_services_;

  // Status monitoring
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::map<std::string, bool> calibration_in_progress_;
  std::map<std::string, bool> homing_in_progress_;
};

}  // namespace arctos_services
}  // namespace arctos_interface

#endif  // ARCTOS_SERVICES_HPP_