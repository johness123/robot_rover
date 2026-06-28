#include "CommanderEngine.hpp"
#include <chrono>

CommanderEngine::CommanderEngine(uint16_t udp_port, const std::string &serial_port, unsigned int baud_rate, const RoverConfig &config)
    : m_receiver(m_io_context, udp_port, ROVER_MAGIC_KEY, ROVER_HANDSHAKE_KEY),
      m_serial_bridge(m_io_context, serial_port, baud_rate),
      m_solver(config),
      m_config(config),
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
    bool was_connected = false;

    while (m_running)
    {
        auto next_tick_time = std::chrono::steady_clock::now() + TICK_INTERVAL;

        // --- STEP 1: WATCHDOG (INTERVENE NETWORK STATE) ---
        // If connected, evaluate whether the signal has been lost for more than 3s
        if (m_receiver.get_state() == ReceiverState::CONNECTED)
        {
            if (m_receiver.is_connection_lost(TOTAL_CONNECTION_LOSS_TIMEOUT))
            {
                std::cout << "[SYSTEM] Watchdog triggered (3000ms). Terminating dead link.\n";
                // This command discards the CONNECTED state, reverting to DISCOVERING
                m_receiver.reset_connection();
            }
        }

        // --- STEP 2: LATCH STATE (After Watchdog may have terminated) ---
        bool is_connected = (m_receiver.get_state() == ReceiverState::CONNECTED);

        // --- STEP 3: CAMERA COORDINATOR (RUNS ONLY ONCE UPON STATE TRANSITION) ---
        if (is_connected && !was_connected)
        {
            // State: Just connected
            std::string pc_ip = m_receiver.get_client_ip();
            if (!pc_ip.empty())
            {
                std::cout << "[SYSTEM] Handshake locked. Routing video to " << pc_ip << "\n";
                m_video_stream.start(pc_ip, m_config.camera_commander_port);
            }
            was_connected = true; // Lock flag, subsequent iterations will not enter here anymore
        }
        else if (!is_connected && was_connected)
        {
            // State: Just disconnected
            m_video_stream.stop();
            was_connected = false; // Lock flag, subsequent iterations will not waste CPU calling stop() anymore
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