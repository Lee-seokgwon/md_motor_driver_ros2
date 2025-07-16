// SerialComm.cpp
#include "md_motor_driver/SerialComm.hpp"
#include <boost/asio/serial_port_base.hpp>
#include <iostream>
#include <sys/ioctl.h>

SerialComm::SerialComm()
: serial_(io_)
{
}

SerialComm::~SerialComm()
{
    if (serial_.is_open()) {
        serial_.close();
    }
}

bool SerialComm::open(const std::string& port, unsigned int baudrate)
{
    try {
        serial_.open(port);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        return true;
    } catch (boost::system::system_error& e) {
        std::cerr << "[SerialComm] Failed to open serial port: " << e.what() << std::endl;
        return false;
    }
}

bool SerialComm::write(const uint8_t* data, size_t length)
{
    if (!serial_.is_open()) {
        return false;
    }

    try {
        size_t bytes_written = boost::asio::write(serial_, boost::asio::buffer(data, length));
        return bytes_written == length;
    } catch (boost::system::system_error& e) {
        std::cerr << "[SerialComm] Write error: " << e.what() << std::endl;
        return false;
    }
}

size_t SerialComm::read(uint8_t* buffer, size_t max_length)
{
    if (!serial_.is_open()) {
        return 0;
    }

    try {
        return serial_.read_some(boost::asio::buffer(buffer, max_length));
    } catch (boost::system::system_error& e) {
        std::cerr << "[SerialComm] Read error: " << e.what() << std::endl;
        return 0;
    }
}

bool SerialComm::isOpen() const
{
    return serial_.is_open();
}

size_t SerialComm::available()
{
    if (!serial_.is_open()) {
        return 0;
    }

    int bytes_available = 0;
    ioctl(serial_.native_handle(), FIONREAD, &bytes_available);
    return static_cast<size_t>(bytes_available);
}