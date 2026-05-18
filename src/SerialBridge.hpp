#ifndef SERIAL_BRIDGE_HPP
#define SERIAL_BRIDGE_HPP

#include <asio.hpp>
#include <vector>
#include <cstdint>
#include "./DTO/Command.hpp"

/**
 * @class SerialBridge
 * @brief Manages hardware UART transmission and protocol serialization.
 *
 * Wraps the ASIO serial port implementation to interface with the Arduino.
 * Responsible for framing strictly-aligned DTOs with synchronization bytes
 * and computing integrity checksums prior to transmission.
 */
class SerialBridge
{
public:
    /**
     * @brief Initializes the UART interface with strict hardware parameters.
     *
     * @param io_context Shared ASIO execution context.
     * @param device_path Absolute path to the UART device (e.g., "/dev/ttyAMA0").
     * @param baud_rate Serial communication speed (e.g., 115200).
     */
    SerialBridge(asio::io_context &io_context, const std::string &device_path, unsigned int baud_rate);

    /**
     * @brief Serializes and writes a WheelActorCommand DTO to the serial buffer.
     *
     * @param cmd The populated command structure retrieved from the ActorStateBuffer.
     */
    void transmit_wheel_command(const WheelActorCommand &cmd);

private:
    asio::serial_port m_serial;

    /**
     * @brief Computes an XOR-based checksum for payload validation.
     *
     * @param payload Raw byte pointer to the data segment.
     * @param length Exact byte count of the payload.
     * @return uint8_t The calculated parity byte.
     */
    uint8_t calculate_checksum(const uint8_t *payload, std::size_t length) const;
};

#endif // SERIAL_BRIDGE_HPP