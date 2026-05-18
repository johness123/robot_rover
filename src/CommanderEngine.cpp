#include "CommanderEngine.hpp"
#include <chrono>

CommanderEngine::CommanderEngine(uint16_t udp_port, const std::string &serial_port, unsigned int baud_rate)
    : m_receiver(m_io_context, udp_port),
      m_serial_bridge(m_io_context, serial_port, baud_rate),
      m_running(false)
{
    m_receiver.register_handler(0x01, [this](const uint8_t *data, std::size_t size)
                                { m_wheel_buffer.update_from_network(data, size); });

    m_receiver.register_handler(0x02, [this](const uint8_t *data, std::size_t size)
                                { m_camera_buffer.update_from_network(data, size); });
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
    constexpr auto TICK_INTERVAL = std::chrono::milliseconds(50);
    constexpr auto NETWORK_TIMEOUT = std::chrono::milliseconds(250);

    while (m_running)
    {
        auto next_tick_time = std::chrono::steady_clock::now() + TICK_INTERVAL;

        if (m_wheel_buffer.is_stale(NETWORK_TIMEOUT))
        {
            WheelActorCommand emergency_cmd = {0};
            emergency_cmd.actor_id = 0x01;
            emergency_cmd.motor_power = 0;
            emergency_cmd.emergency_brake = 1;

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