#include "./SerialBridge.hpp"

SerialBridge::SerialBridge(asio::io_context &io_context, const std::string &device_path, unsigned int baud_rate)
    : m_serial(io_context, device_path)
{
    m_serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
    m_serial.set_option(asio::serial_port_base::character_size(8));
    m_serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    m_serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    m_serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
}

uint8_t SerialBridge::calculate_checksum(const uint8_t *payload, std::size_t length) const
{
    uint8_t checksum = 0;
    for (std::size_t i = 0; i < length; ++i)
    {
        checksum ^= payload[i];
    }
    return checksum;
}

void SerialBridge::transmit_wheel_command(const WheelActorCommand &cmd)
{
    std::vector<uint8_t> frame;
    frame.reserve(sizeof(WheelActorCommand) + 3);

    frame.push_back(0xAA);

    const uint8_t *cmd_ptr = reinterpret_cast<const uint8_t *>(&cmd);
    for (std::size_t i = 0; i < sizeof(WheelActorCommand); ++i)
    {
        frame.push_back(cmd_ptr[i]);
    }

    frame.push_back(calculate_checksum(cmd_ptr, sizeof(WheelActorCommand)));

    frame.push_back(0x55);

    asio::write(m_serial, asio::buffer(frame));
}