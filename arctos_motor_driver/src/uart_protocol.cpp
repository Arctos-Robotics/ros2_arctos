#include "arctos_motor_driver/uart_protocol.hpp"
// #include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <stdexcept>

namespace {
// ===== Helpers nội bộ (không thay đổi API) =====
inline void trim(std::string &s) {
  auto not_space = [](unsigned char ch){ return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

inline std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> tokens;
  std::string item;
  std::stringstream ss(s);
  while (std::getline(ss, item, delim)) {
    tokens.push_back(item);
  }
  return tokens;
}

inline bool parseDoubleStrict(const std::string &token_in, double &out) {
  std::string token = token_in;
  trim(token);
  if (token.empty()) return false;

  // Chấp nhận dấu phẩy làm dấu thập phân
  std::replace(token.begin(), token.end(), ',', '.');

  char *endptr = nullptr;
  errno = 0;
  const double val = std::strtod(token.c_str(), &endptr);
  if (errno != 0 || endptr == token.c_str()) {
    return false;
  }
  // Không chấp nhận rác sau số (ngoại trừ khoảng trắng)
  while (*endptr != '\0') {
    if (!std::isspace(static_cast<unsigned char>(*endptr))) return false;
    ++endptr;
  }
  out = val;
  return true;
}

// EOL dùng khi đọc/ghi dòng. Nếu firmware yêu cầu CRLF, đổi thành "\r\n".
constexpr const char* kEOL = "\r";

// Lấy ký tự phân tách đầu tiên từ macro DELIMITER (giả định 1 ký tự)
constexpr char delimChar() { return DELIMITER[0]; }

} // anonymous namespaceS


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
    try {
        if (serial_conn_.isOpen()) {
            serial_conn_.close();
            std::cerr << "UART: Closed existing connection on " << serial_device << '\n';
        }
        serial_conn_.setPort(serial_device);
        serial_conn_.setBaudrate(static_cast<uint32_t>(baud_rate));
        serial::Timeout tt = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms));
        serial_conn_.setTimeout(tt); // setTimeout yêu cầu tham chiếu
        serial_conn_.open();
        if (serial_conn_.isOpen()) {
            std::cerr << "UART: Opened connection on " << serial_device
                  << " at " << baud_rate << " bps with timeout " << timeout_ms << " ms\n";
        }
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("UART setup failed: ") + e.what());
    }
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
 * @param data The raw message string received from the UART device, the string format is "j1,j2,j3,j4,j5,j6"
 * @return std::vector<double> A vector containing decoded position values for each joint
 * 
 * @note This function currently returns an empty vector - implementation needed
 *       based on the specific protocol format used by the connected device.
 */
std::vector<double> UartProtocol::decodeMessage(const std::string data) {
    // decode the data here
    // Làm việc trên bản sao (đúng prototype nhận by-value)
    std::string line = data;

    // Bỏ EOL/CR ở cuối nếu có
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
        line.pop_back();
    }

    // Tách theo DELIMITER, loại token rỗng và trim
    std::vector<std::string> toks = split(line, delimChar());
    std::vector<std::string> cleaned;
    cleaned.reserve(toks.size());
    for (auto &t : toks) {
        trim(t);
        if (!t.empty()) cleaned.push_back(t);
    }

    if (cleaned.size() != 6) {
        std::ostringstream oss;
        oss << "decodeMessage: expected 6 values, got " << cleaned.size()
            << " (data='" << line << "')";
        throw std::runtime_error(oss.str());
    }

    std::vector<double> axes(6, 0.0);
    for (size_t i = 0; i < 6; ++i) {
        if (!parseDoubleStrict(cleaned[i], axes[i])) {
            std::ostringstream oss;
            oss << "decodeMessage: invalid number at index " << i
                << " ('" << cleaned[i] << "')";
            throw std::runtime_error(oss.str());
        }
    }
    return axes;
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
 if (positions.size() != 6) {
        std::cerr << "sendPosition: positions must have 6 elements, got "
                  << positions.size() << std::endl;
        return false;
    }

    std::ostringstream oss;
    oss << std::setprecision(10) << positions[0];
    for (size_t i = 1; i < 6; ++i) {
        oss << DELIMITER << std::setprecision(10) << positions[i];
    }
    oss << kEOL;

    return sendMsg(oss.str());
}

/**
 * @brief Reads incoming data from UART and stores it in the receive buffer.
 * 
 * This function continuously reads data from the serial connection until it encounters
 * the specified delimiter (defined as "," in DELIMITER). The received data is then
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
    if (!this->connected()) {
        return;
    }
    try {
        // serial::readline sẽ đọc tới ký tự cuối trong kEOL (ở đây là '\n')
        std::string raw = serial_conn_.readline(65535UL, kEOL);

        // Cắt CR/LF đuôi nếu có
        while (!raw.empty() && (raw.back() == '\n' || raw.back() == '\r')) {
            raw.pop_back();
        }

        if (!raw.empty()) {
            rev_buffer_.push(raw);
            std::cerr << "uart: readToBuffer: received: " << raw << '\n';
        }
        else{
            std::cerr << ".";
        }
    } catch (const std::exception& e) {
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

}// namespace arctos_motor_driver
