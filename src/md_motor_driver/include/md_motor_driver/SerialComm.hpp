// SerialComm.hpp
#pragma once
#include <boost/asio.hpp>
#include <string>

class SerialComm {
public:
    SerialComm();
    ~SerialComm();

    bool open(const std::string& port, unsigned int baudrate);
    bool writeData(const uint8_t* data, size_t length);
    size_t readData(uint8_t* buffer, size_t max_length);
    bool isOpen() const;
    size_t available();

private:
    boost::asio::io_service io_;
    boost::asio::serial_port serial_;
};
