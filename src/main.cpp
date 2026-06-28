/**
 * @file main.cpp
 * @brief Application entry point for the Rover Brain central processing node.
 *
 * Bootstraps the execution environment, parses runtime configurations, and
 * instantiates the CommanderEngine. Implements POSIX signal handling to
 * guarantee graceful degradation and hardware teardown upon termination.
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "./CommanderEngine.hpp"
#include "./RoverConfig.hpp"

///< Global pointer required to map POSIX asynchronous OS signals to class methods.
CommanderEngine *g_engine_ptr = nullptr;

/**
 * @brief Asynchronous interrupt handler capturing OS-level termination requests.
 * @param signum The POSIX signal identifier (e.g., SIGINT from Ctrl+C).
 */
void signal_handler(int signum)
{
    std::cout << "\n[SYSTEM] Intercepted termination signal (" << signum << "). Initiating graceful shutdown...\n";
    if (g_engine_ptr)
    {
        g_engine_ptr->stop(); ///< Safely tear down network threads and release UART file descriptors.
    }
}

int main()
{
    std::cout << "=================================================\n";
    std::cout << "          ROVER BRAIN - FIRMWARE v2.5            \n";
    std::cout << "=================================================\n";

    // Register OS signal traps for clean exits (Ctrl+C or kill commands)
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try
    {
        // Absolute hardware path for Pi Zero 2W primary UART
        const std::string UART_DEVICE = "/dev/serial0";
        const uint16_t UDP_LISTEN_PORT = 5005;
        const unsigned int BAUD_RATE = 115200;
        // const std::string CONFIG_PATH = "./rover_config.json";
        const RoverConfig config = RoverConfig::load_from_file("./rover_config.json");
        std::cout << "[SYSTEM] Bootstrapping Commander Engine...\n";
        std::cout << "  -> Network Port : " << UDP_LISTEN_PORT << " (UDP)\n";
        std::cout << "  -> UART Device  : " << UART_DEVICE << " @ " << BAUD_RATE << " bps\n";

        // Instantiate the core orchestrator
        CommanderEngine engine(UDP_LISTEN_PORT, UART_DEVICE, BAUD_RATE, config);

        // Map the global pointer for the signal handler
        g_engine_ptr = &engine;

        std::cout << "[SYSTEM] Engine is LIVE. Entering deterministic hardware loop.\n";
        std::cout << "[SYSTEM] Press Ctrl+C to terminate safely.\n";
        std::cout << "-------------------------------------------------\n";

        // This call blocks the main thread, looping at 50Hz until engine.stop() is called
        engine.start();

        std::cout << "[SYSTEM] Hardware loop terminated. Core dumped safely. Goodbye!\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "[FATAL ERROR] Unhandled system exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}