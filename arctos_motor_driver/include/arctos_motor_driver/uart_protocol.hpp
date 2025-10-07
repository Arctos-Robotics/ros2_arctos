#ifndef UART_PROTOCOL_H_
#define UART_PROTOCOL_H_

#include "serial/serial.h"
#include <string>
#include <vector>

#define DELIMITER   ";"

class UartProtocol
{
public:
    explicit UartProtocol(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {}

    ~UartProtocol() = default;

    void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
    /**
     * @brief This function is used to read a specified string from serial_comm line and store it in receive buffer.
     * 
     */
    void readToBuffer();
    void sendEmptyMsg();
    void readEncoderValues(int &val_1, int &val_2);

    bool connected() const { return serial_conn_.isOpen(); }

    bool sendMsg(const std::string &msg_to_send);


private:
    serial::Serial serial_conn_;
    std::vector<std::string> rev_buffer_;
};


#endif  //URT_PROTOCOL_H_