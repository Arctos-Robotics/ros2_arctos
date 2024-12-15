#include "rclcpp/rclcpp.hpp"
#include "arctos_motor_driver/motor_driver.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arctos_motor_driver::MotorDriver>();
    node->setMotorId(1);
    uint8_t direction = 1; uint16_t speed = 2; uint8_t acceleration = 2;
    node->setVelocity(direction, speed, acceleration);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
