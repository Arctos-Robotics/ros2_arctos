#include "rclcpp/rclcpp.hpp"
#include "arctos_motor_driver/motor_driver.hpp"
#include "rclcpp/executors.hpp"
#include <chrono>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arctos_motor_driver::MotorDriver>();

    // Set motor properties
    node->setMotorId(1);
    uint8_t direction = 1; 
    uint16_t speed = 2; 
    uint8_t acceleration = 2;
    node->setVelocity(direction, speed, acceleration);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    // Start the executor to spin the node
    exec.spin();

    // Ensure the motor stops before shutting down
    RCLCPP_INFO(node->get_logger(), "Stopping motor before shutdown.");
    node->stopMotor();

    // Give some time for the stopMotor command to be processed (Optional: 500 ms delay)
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time) < std::chrono::milliseconds(500)) {
        rclcpp::spin_some(node);  // Process any pending messages, like the stop command
        rclcpp::sleep_for(std::chrono::milliseconds(50));  // Small sleep to allow message processing
    }

    rclcpp::shutdown();  // Shutdown ROS 2
    return 0;
}
