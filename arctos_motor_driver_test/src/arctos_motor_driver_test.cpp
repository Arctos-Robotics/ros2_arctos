#include "rclcpp/rclcpp.hpp"
#include "arctos_motor_driver/motor_driver.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arctos_motor_driver::MotorDriver>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Starting rclcpp::spin... testnode");
    exec.spin();
    RCLCPP_INFO(node->get_logger(), "Exiting rclcpp::spin...");

    rclcpp::shutdown();
    return 0;
}
