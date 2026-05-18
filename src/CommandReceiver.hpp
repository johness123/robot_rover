#ifndef COMMAND_RECEIVER_HPP
#define COMMAND_RECEIVER_HPP

#include <asio.hpp>
#include <array>
#include <functional>
#include <unordered_map>
#include <cstring>
#include "./DTO/Command.hpp"

/**
 * @class CommandReceiver
 * @brief An asynchronous UDP receiver handling raw telemetry and control payloads.
 *
 * This class binds to a specified UDP port and continuously listens for incoming
 * datagrams using the Asio asynchronous event loop. It acts as a payload router,
 * reading the initial actor_id byte from the datagram and invoking the mapped
 * callback function with a safely memory-copied DTO structure.
 */
class CommandReceiver
{
public:
    using GenericCallback = std::function<void(const uint8_t *, std::size_t)>;

    /**
     * @brief Constructs the CommandReceiver and opens the UDP socket.
     *
     * @param io_context Reference to the central Asio I/O execution context.
     * @param port The local port number to bind the receiver.
     */
    CommandReceiver(asio::io_context &io_context, uint16_t port)
        : m_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), port))
    {
        start_receive();
    }

    /**
     * @brief Registers a callback handler for a specific actor ID.
     *
     * @param actor_id The unique identifier for the target hardware subsystem.
     * @param handler A std::function wrapping the memory state update logic.
     */
    void register_handler(uint8_t actor_id, GenericCallback handler)
    {
        m_handlers[actor_id] = std::move(handler);
    }

private:
    /**
     * @brief Initiates an asynchronous receive operation.
     */
    void start_receive()
    {
        m_socket.async_receive_from(
            asio::buffer(m_recv_buffer), m_remote_endpoint,
            [this](const asio::error_code &error, std::size_t bytes_transferred)
            {
                handle_receive(error, bytes_transferred);
            });
    }

    /**
     * @brief The completion callback invoked by Asio when datagrams arrive.
     *
     * @param error Asio error code representing the operation status.
     * @param bytes_transferred The exact number of bytes read from the socket.
     */
    void handle_receive(const asio::error_code &error, std::size_t bytes_transferred)
    {
        if (!error && bytes_transferred > 0)
        {
            uint8_t current_actor_id = m_recv_buffer[0];

            auto it = m_handlers.find(current_actor_id);
            if (it != m_handlers.end())
            {
                it->second(m_recv_buffer.data(), bytes_transferred);
            }
        }

        start_receive();
    }

    asio::ip::udp::socket m_socket;
    asio::ip::udp::endpoint m_remote_endpoint;
    std::array<uint8_t, 1024> m_recv_buffer;
    std::unordered_map<uint8_t, GenericCallback> m_handlers;
};

#endif // COMMAND_RECEIVER_HPP