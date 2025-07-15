// SerialComm.cpp
#include "md_motor_driver/SerialComm.hpp"
#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

SerialComm::SerialComm() : serial_(io_) {}

SerialComm::~SerialComm() {
    if (serial_.is_open()) serial_.close();
}

bool SerialComm::open(const std::string& port, unsigned int baudrate) {
    boost::system::error_code ec;
    serial_.open(port, ec);
    if (ec) {
        std::cerr << "Failed to open port: " << ec.message() << std::endl;
        return false;
    }

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    return true;
}

bool SerialComm::writeData(const uint8_t* data, size_t length) {
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(data, length), ec);
    return !ec;
}

size_t SerialComm::readData(uint8_t* buffer, size_t max_length) {
    boost::system::error_code ec;
    return serial_.read_some(boost::asio::buffer(buffer, max_length), ec);
}

bool SerialComm::isOpen() const {
    return serial_.is_open();
}

size_t SerialComm::available(){
    int bytes_available = 0;
    if (serial_.is_open()) {
        ioctl(serial_.native_handle(), FIONREAD, &bytes_available);
    }
    return static_cast<size_t>(bytes_available);
}