#include "arctos_motor_driver/uart_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

namespace arctos_motor_driver {

/**
 * @brief Initializes and configures the UART serial connection.
 * 
 * This function sets up the serial communication parameters including the device port,
 * baud rate, and timeout settings. It then opens the serial connection for communication.
 * 
 * @param serial_device The device path for the serial port (e.g., "/dev/ttyUSB0")
 * @param baud_rate The communication speed in bits per second (e.g., 9600, 115200)
 * @param timeout_ms The timeout value in milliseconds for read/write operations
 * 
 * @throws serial::SerialException if the serial port cannot be opened or configured
 */
void UartProtocol::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
}

/**
 * @brief Sends an empty message (carriage return) over UART.
 * 
 * This function is typically used to wake up the connected device, request status,
 * or serve as a keep-alive signal. It sends a carriage return character ("\r").
 * 
 * @return true if the message was sent successfully, false otherwise
 */
bool UartProtocol::sendEmptyMsg()
{
    return sendMsg("\r");
}

/**
 * @brief Decodes a received UART message string into position values.
 * 
 * This function parses the incoming message string and extracts position data
 * for multiple joints/motors. The expected format should be defined based on
 * the communication protocol (e.g., comma-separated, semicolon-separated values).
 * 
 * @param data The raw message string received from the UART device
 * @return std::vector<double> A vector containing decoded position values for each joint
 * 
 * @note This function currently returns an empty vector - implementation needed
 *       based on the specific protocol format used by the connected device.
 */
std::vector<double> UartProtocol::decodeMessage(std::string data) {
    // decode the data here
    return std::vector<double>();
}

/**
 * @brief Sends position commands to multiple joints/motors via UART.
 * 
 * This function takes a vector of position values and formats them into a proper
 * message string according to the communication protocol. The message is then
 * sent to the connected device over the UART connection.
 * 
 * @param positions Reference to a vector containing position values for each joint/motor
 * @return true if the position command was sent successfully, false otherwise
 * 
 * @note The message construction needs to be implemented based on the specific
 *       protocol format expected by the connected motor controller/device.
 */
bool UartProtocol::sendPosition(std::vector<double> &positions) {
    std::string message;
    // construct the message here
    return sendMsg(message);
}

/**
 * @brief Reads incoming data from UART and stores it in the receive buffer.
 * 
 * This function continuously reads data from the serial connection until it encounters
 * the specified delimiter (defined as ";" in DELIMITER). The received data is then
 * stored in a queue buffer for later processing. This function is typically called
 * in a loop or timer to continuously monitor incoming messages.
 * 
 * @note The function will return immediately if the serial connection is not established.
 *       Any exceptions during reading (e.g., timeout, disconnection) are caught and logged.
 * 
 * @see getFromBuffer() to retrieve messages from the buffer
 */
void UartProtocol::readToBuffer(void)
{
    if (!this->connected())
    {
        //std::wcerr << "uart: readToBuffer: Serial not connected!\n";
        return;
    }
    try
    {
        std::string data = serial_conn_.readline(65535UL, DELIMITER);
        rev_buffer_.push(data);
    }
    catch(const std::exception& e)
    {
        std::cerr << "uart: readToBuffer: " << e.what() << '\n';
    }
}

/**
 * @brief Retrieves and removes the oldest message from the receive buffer.
 * 
 * This function implements a FIFO (First In, First Out) approach to message retrieval.
 * It returns the oldest message stored in the buffer and removes it from the queue.
 * If the buffer is empty, it returns an empty string.
 * 
 * @return std::string The oldest message from the buffer, or empty string if buffer is empty
 * 
 * @note This function should be called after readToBuffer() has been used to populate
 *       the buffer with incoming messages. Check the return value to determine if a
 *       valid message was retrieved.
 * 
 * @see readToBuffer() to populate the buffer with incoming messages
 */
std::string UartProtocol::getFromBuffer(void) 
{
    if (!rev_buffer_.empty()) 
    {
        std::string data = rev_buffer_.front();
        rev_buffer_.pop();
        return data;
    }
    return "";
}

/**
 * @brief Sends a message string over the UART connection.
 * 
 * This is a private helper function that handles the low-level transmission of
 * messages over the serial connection. It checks the connection status before
 * attempting to send and handles any exceptions that may occur during transmission.
 * 
 * @param msg_to_send The message string to transmit over UART
 * @return true if the message was sent successfully, false if connection is down or error occurred
 * 
 */
bool UartProtocol::sendMsg(const std::string &msg_to_send)
{
    if (!this->connected())
    {
        //std::wcerr << "uart: sendMsg: Serial not connected!\n";
        return false;
    }
    try
    {
        serial_conn_.write(msg_to_send);
    }
    catch(const std::exception& e)
    {
        std::cerr << "uart: sendMsg: " << e.what() << '\n';
        return false;
    }
    return true;
}

}