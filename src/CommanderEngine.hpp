#ifndef COMMANDER_ENGINE_HPP
#define COMMANDER_ENGINE_HPP

#include <asio.hpp>
#include <thread>
#include <atomic>
#include <string>
#include "./CommandReceiver.hpp"
#include "./ActorStateBuffer.hpp"
#include "./SerialBridge.hpp"
#include "./KinematicSolver.hpp"
#include "./RoverConfig.hpp"
#include "./DTO/Command.hpp"
#include "./VideoStreamManager.hpp"

/**
 * @class CommanderEngine
 * @brief Orchestrates the lifecycle and multithreaded execution of rover-brain subsystems.
 *
 * This core engine initializes network listeners with handshake topology, instantiates
 * actor state buffers, and configures the hardware serial bridge. It enforces thread isolation
 * by spawning a dedicated network thread for asynchronous Asio events while maintaining a strict
 * deterministic polling loop on the main thread to dispatch kinematic updates to the MCU.
 */
class CommanderEngine
{
public:
    /**
     * @brief Constructor initializing all subsystems and loading mechanical configurations.
     * @param udp_port Local UDP port dedicated for commander host discovery and communication.
     * @param serial_port System file path targeting the physical UART peripheral driver.
     * @param baud_rate Serial transmission speed assigned to the UART interface.
     * @param config_path System file path pointing to the JSON configuration file.
     */
    CommanderEngine(uint16_t udp_port, const std::string &serial_port, unsigned int baud_rate, const RoverConfig &config);

    /**
     * @brief Destructor ensuring safe teardown of threads and socket allocations.
     */
    ~CommanderEngine();

    /**
     * @brief Spawns the isolated network thread and blocks the caller with the hardware tick loop.
     */
    void start();

    /**
     * @brief Terminates background processing and signals execution loops to tear down.
     */
    void stop();

private:
    /**
     * @brief Deterministic high-priority loop enforcing real-time execution bounds.
     */
    void hardware_tick_loop();

    asio::io_context m_io_context; ///< Shared asynchronous execution context for network and serial I/O.
    CommandReceiver m_receiver;    ///< Authenticated network receiver managing UDP topologies.
    SerialBridge m_serial_bridge;  ///< Hardware abstraction layer serialization bridge interfacing with the MCU.
    KinematicSolver m_solver;      ///< Kinematics translation block converting high-level intents to joint outputs.
    RoverConfig m_config;
    ActorStateBuffer<HardwareWheelCommand> m_wheel_buffer; ///< Thread-safe buffer housing pending physical joint commands.

    VideoStreamManager m_video_stream; ///< Orchestrator for the hardware-accelerated FPV camera.

    std::thread m_network_thread; ///< Dedicated worker isolation thread processing incoming network packets.
    std::atomic<bool> m_running;  ///< Core execution lifecycle flag governing loop continuation constraints.

    static constexpr uint32_t ROVER_MAGIC_KEY = 0x524F5652;     ///< Validation passcode matching network specifications ("ROVR").
    static constexpr uint32_t ROVER_HANDSHAKE_KEY = 0x484E4453; ///< Handshake discovery passcode constraint token ("HNDS").
};

#endif // COMMANDER_ENGINE_HPP