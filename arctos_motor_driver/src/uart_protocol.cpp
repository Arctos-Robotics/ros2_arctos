#include "arctos_motor_driver/uart_protocol.hpp"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


void UartProtocol::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));
}


void UartProtocol::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void UartProtocol::readToBuffer(void)
{
    std::string data = serial_conn_.readline(65535UL, DELIMITER);
    rev_buffer_.push_back(data);
}

bool UartProtocol::sendMsg(const std::string &msg_to_send)
{
    try
    {
        serial_conn_.write(msg_to_send);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    return true;
}