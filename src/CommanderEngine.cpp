#include "CommanderEngine.hpp"
#include <chrono>

CommanderEngine::CommanderEngine(uint16_t udp_port, const std::string &serial_port, unsigned int baud_rate, const std::string &config_path)
    : m_receiver(m_io_context, udp_port, ROVER_MAGIC_KEY, ROVER_HANDSHAKE_KEY),
      m_serial_bridge(m_io_context, serial_port, baud_rate),
      m_solver(RoverConfig::load_from_file(config_path)),
      m_running(false)
{
    // Register the intent callback handler for the chassis actor (0x01)
    m_receiver.register_intent_handler(0x01, [this](const RoverIntentCommand &cmd)
                                       {
                                           // Execute localized kinematic translation immediately on network ingress
                                           HardwareWheelCommand hw_cmd = m_solver.solve(cmd);
                                           
                                           // Commit calculated low-level joint states into the thread-safe state buffer
                                           m_wheel_buffer.update_from_network(reinterpret_cast<const uint8_t*>(&hw_cmd), sizeof(HardwareWheelCommand)); });
}

CommanderEngine::~CommanderEngine()
{
    stop();
}

void CommanderEngine::start()
{
    m_running = true;
    m_network_thread = std::thread([this]()
                                   { m_io_context.run(); });

    hardware_tick_loop();
}

void CommanderEngine::stop()
{
    m_running = false;
    m_io_context.stop();
    if (m_network_thread.joinable())
    {
        m_network_thread.join();
    }
}

void CommanderEngine::hardware_tick_loop()
{
    // UPDATE: 50Hz execution rate (1000ms / 50Hz = 20ms per hardware tick)
    constexpr auto TICK_INTERVAL = std::chrono::milliseconds(20);

    // Failsafe triggers remain the same (250ms = 12.5 missed packets at 50Hz)
    constexpr auto NETWORK_TIMEOUT = std::chrono::milliseconds(250);
    constexpr auto TOTAL_CONNECTION_LOSS_TIMEOUT = std::chrono::milliseconds(3000);

    while (m_running)
    {
        auto next_tick_time = std::chrono::steady_clock::now() + TICK_INTERVAL;

        // Critical Link Check 1: Evaluate absolute connection loss threshold for topology restoration
        if (m_receiver.get_state() == ReceiverState::CONNECTED)
        {
            if (m_receiver.is_connection_lost(TOTAL_CONNECTION_LOSS_TIMEOUT))
            {
                std::cout << "[SYSTEM] Watchdog triggered (3000ms). Terminating dead link.\n";
                m_receiver.reset_connection();
            }
        }

        // Critical Link Check 2: Evaluate transient packet staleness for deterministic failsafe execution
        if (m_wheel_buffer.is_stale(NETWORK_TIMEOUT))
        {
            HardwareWheelCommand emergency_cmd = {0};
            emergency_cmd.actor_id = 0x01;
            emergency_cmd.emergency_brake = 1;
            // All localized wheel power parameters are initialized to zero implicitly via structure aggregation

            m_serial_bridge.transmit_wheel_command(emergency_cmd);
        }
        else
        {
            auto wheel_cmd = m_wheel_buffer.consume();
            if (wheel_cmd.has_value())
            {
                m_serial_bridge.transmit_wheel_command(wheel_cmd.value());
            }
        }

        std::this_thread::sleep_until(next_tick_time);
    }
}