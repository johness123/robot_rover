#pragma once

#include <asio.hpp>
#include <array>
#include <functional>
#include <unordered_map>
#include <iostream>
#include <cstring>
#include "DTO/Command.hpp"
#include <chrono>

#define ACK_ 0x41434B5F

/**
 * @enum ReceiverState
 * @brief Enumeration tracking the active network topology state of the socket.
 */
enum class ReceiverState : uint8_t
{
    DISCOVERING, ///< Open socket listening to omnidirectional broadcast/multicast endpoints.
    CONNECTED    ///< Restricted kernel-level connection associated explicitly with a verified commander host.
};

/**
 * @class CommandReceiver
 * @brief Network subsystem managing UDP packet ingestion, hardware handshakes, and cryptographic verification.
 */
class CommandReceiver
{
public:
    using IntentCallback = std::function<void(const RoverIntentCommand &)>; ///< Standard callback signature for chassis intents.

    /**
     * @brief Constructor allocating the network socket and initializing cryptographic keys.
     * @param io_context Shared asynchronous execution pipeline context.
     * @param port Local host UDP port assigned for binding.
     * @param magic_key Verification passphrase allocated for the drive loop.
     * @param handshake_key Verification token allocated for initial host discovery.
     */
    CommandReceiver(asio::io_context &io_context, uint16_t port, uint32_t magic_key, uint32_t handshake_key)
        : m_socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), port)),
          m_port(port),
          m_state(ReceiverState::DISCOVERING),
          m_magic_key(magic_key),
          m_handshake_key(handshake_key),
          m_latest_sequence(0),
          m_last_activity(std::chrono::steady_clock::now())
    {
        start_receive();
    }

    /**
     * @brief Maps an operational callback handler to a distinct subsystem actor identity.
     */
    void register_intent_handler(uint8_t actor_id, IntentCallback handler)
    {
        m_handlers[actor_id] = std::move(handler);
    }

    /**
     * @brief Forces tearing down the existing remote connection and reverts the socket back to discovering state.
     * @note Closes and re-binds the socket to clear kernel-level remote endpoint affinity.
     */
    void reset_connection()
    {
        asio::error_code ec;
        m_socket.close(ec);
        m_socket.open(asio::ip::udp::v4(), ec);
        m_socket.set_option(asio::socket_base::reuse_address(true), ec);
        m_socket.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), m_port), ec);

        m_state = ReceiverState::DISCOVERING;
        m_latest_sequence = 0;

        std::cout << "[NET] Connection cleared. Reverting to DISCOVERING mode on port " << m_port << "\n";
        start_receive();
    }

    /**
     * @brief Inline public accessor exposing the current connectivity topology status.
     */
    ReceiverState get_state() const { return m_state; }

    bool is_connection_lost(std::chrono::milliseconds timeout) const
    {
        return (std::chrono::steady_clock::now() - m_last_activity) > timeout;
    }

private:
    /**
     * @brief Triggers the underlying operating system kernel to capture asynchronous incoming UDP datagrams.
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
     * @brief Asynchronous handler processing network event completion signals.
     */
    void handle_receive(const asio::error_code &error, std::size_t bytes_transferred)
    {
        if (!error && bytes_transferred > 0)
        {
            process_packet(bytes_transferred);
        }
        start_receive();
    }

    /**
     * @brief Evaluates incoming structural patterns depending on the current subsystem execution state.
     */
    void process_packet(std::size_t size)
    {
        if (m_state == ReceiverState::DISCOVERING)
        {
            if (size != sizeof(RoverIncomingHandshake))
                return;

            RoverIncomingHandshake handshake;
            std::memcpy(&handshake, m_recv_buffer.data(), sizeof(RoverIncomingHandshake));

            if (handshake.handshake_key == m_handshake_key)
            {
                asio::error_code ec;
                m_socket.connect(m_remote_endpoint, ec); ///< Lock socket filters exclusively to this remote host.
                if (!ec)
                {
                    m_state = ReceiverState::CONNECTED;

                    // uint32_t ack_token = ACK_; ///< Response verification pattern ("ACK_").
                    m_last_activity = std::chrono::steady_clock::now();
                    m_socket.send(asio::buffer(&handshake, sizeof(RoverIncomingHandshake)), 0, ec);

                    std::cout << "[NET] Authenticated handshake verified. Associated with remote host: "
                              << m_remote_endpoint.address().to_string() << ":" << m_remote_endpoint.port() << "\n";

                    verify_vehicle_components();
                }
            }
        }
        else if (m_state == ReceiverState::CONNECTED)
        {
            if (size != sizeof(RoverIntentCommand))
                return;

            RoverIntentCommand cmd;
            std::memcpy(&cmd, m_recv_buffer.data(), sizeof(RoverIntentCommand));

            if (cmd.header.magic_key != m_magic_key)
                return;

            if (m_latest_sequence > 0 && cmd.header.sequence_num <= m_latest_sequence)
                return;
            m_latest_sequence = cmd.header.sequence_num;
            m_last_activity = std::chrono::steady_clock::now();

            auto it = m_handlers.find(cmd.actor_id);
            if (it != m_handlers.end())
            {
                it->second(cmd);
            }
        }
    }

    /**
     * @brief Empty placeholder function reserved for structural hardware diagnostic validation routines.
     * @todo Implement full peripheral loopback and thermal state diagnostics in subsequent sprints.
     */
    void verify_vehicle_components()
    {
        // Placeholder for future multi-component diagnostics integration.
    }

    asio::ip::udp::socket m_socket;            ///< Hardware socket encapsulation mapping kernel network events.
    asio::ip::udp::endpoint m_remote_endpoint; ///< Dynamic memory address tracking active network transaction endpoints.
    std::array<uint8_t, 1024> m_recv_buffer;   ///< Local memory layout workspace holding transient packet frames.

    uint16_t m_port;                                        ///< Local cache of host network socket binding port configurations.
    ReceiverState m_state;                                  ///< Runtime system operational tracking vector flag.
    uint32_t m_magic_key;                                   ///< Internal runtime drive system security check passcode.
    uint32_t m_handshake_key;                               ///< Discovery validation passcode used for host enrollment constraints.
    uint32_t m_latest_sequence;                             ///< Storage tracking current replay mitigation ceiling values.
    std::unordered_map<uint8_t, IntentCallback> m_handlers; ///< Registry associative array distributing tasks to targets.
    std::chrono::steady_clock::time_point m_last_activity;  ///< Temporal marker of the last valid network packet.
};