#ifndef COMMANDER_ENGINE_HPP
#define COMMANDER_ENGINE_HPP

#include <asio.hpp>
#include <thread>
#include <atomic>
#include "./CommandReceiver.hpp"
#include "./ActorStateBuffer.hpp"
#include "./SerialBridge.hpp"
#include "./DTO/Command.hpp"

/**
 * @class CommanderEngine
 * @brief Orchestrates the lifecycle and multithreaded execution of rover-brain subsystems.
 *
 * This core engine initializes network listeners, instantiates actor state buffers,
 * and configures the hardware serial bridge. It enforces thread isolation by spawning
 * a dedicated network thread for asynchronous Asio events while maintaining a strict
 * deterministic polling loop on the main thread to dispatch state updates to the MCU.
 */
class CommanderEngine
{
public:
    CommanderEngine(uint16_t udp_port, const std::string &serial_port, unsigned int baud_rate);
    ~CommanderEngine();

    void start();
    void stop();

private:
    void hardware_tick_loop();

    asio::io_context m_io_context;
    CommandReceiver m_receiver;
    SerialBridge m_serial_bridge;

    ActorStateBuffer<WheelActorCommand> m_wheel_buffer;
    ActorStateBuffer<CameraMountCommand> m_camera_buffer;

    std::thread m_network_thread;
    std::atomic<bool> m_running;
};

#endif // COMMANDER_ENGINE_HPP