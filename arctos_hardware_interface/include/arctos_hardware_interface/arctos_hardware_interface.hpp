#ifndef ARCTOS_HARDWARE_INTERFACE_HPP
#define ARCTOS_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "arctos_hardware_interface/ServoManager.hpp"

#include <unordered_map>
#include <vector>
#include <memory>

class ArctosHardwareInterface : public hardware_interface::SystemInterface {
public:
    // Lifecycle methods
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Wrapper for each servo's data
    struct ServoWrapper {
        std::shared_ptr<ServoManager> manager;
        double current_angle{0.0};
        double current_velocity{0.0};
        double target_angle{0.0};
        double target_velocity{0.0};
    };

    // Map CAN IDs to servo data
    std::unordered_map<uint8_t, ServoWrapper> servo_map_;

    // Keep a consistent ordering of joints (so state/command interfaces align)
    std::vector<uint8_t> servo_order_;

    // Motion defaults
    double speed_ = 100;    // default speed for all motors
    uint8_t accel_ = 10;    // default acceleration for all motors

    // CAN socket
    int can_socket_ = -1;

    // Simulated CAN read/write functions
    can_frame receiveCAN();
    void sendCAN(const can_frame& frame);
};

#endif  // ARCTOS_HARDWARE_INTERFACE_HPP
