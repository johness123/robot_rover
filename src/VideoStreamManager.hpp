#pragma once

#include <string>
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sys/wait.h>
#include <cstdint>

/**
 * @class VideoStreamManager
 * @brief Orchestrates the hybrid rpicam-vid and GStreamer pipeline via POSIX subprocesses.
 *
 * Utilizes fork/exec semantics and Process Group IDs (PGID) to spawn and safely terminate
 * piped shell commands without leaving orphaned zombie processes blocking the CSI camera hardware.
 */
class VideoStreamManager
{
public:
    VideoStreamManager() : m_child_pid(-1) {}

    /**
     * @brief Terminates the video stream securely upon destruction.
     */
    ~VideoStreamManager()
    {
        stop();
    }

    /**
     * @brief Spawns a detached background process executing the hybrid video pipeline.
     * @param target_ip The destination IPv4 address of the Windows control station.
     * @param target_port The destination UDP port to transmit the RTP payload.
     * @return true If the child process was successfully spawned.
     */
    bool start(const std::string &target_ip, uint16_t target_port)
    {
        if (m_child_pid > 0)
        {
            std::cerr << "[VIDEO WARN] Stream is already active.\n";
            return false;
        }

        // The battle-tested hybrid pipeline for Debian Trixie / Bookworm
        std::string pipeline = "rpicam-vid -t 0 --inline --width 800 --height 600 --framerate 30 --codec h264 -o - | "
                               "gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! "
                               "udpsink host=" +
                               target_ip + " port=" + std::to_string(target_port);

        pid_t pid = fork(); ///< Clone the current OS process.

        if (pid == -1)
        {
            std::cerr << "[VIDEO FATAL] OS failed to fork process.\n";
            return false;
        }
        else if (pid == 0)
        {
            // --- CHILD PROCESS CONTEXT ---

            // CRITICAL: Detach this process into its own Process Group.
            // This ensures that when we send a kill signal later, it hits /bin/sh AND all piped commands.
            setpgid(0, 0);

            // Replace the child's memory space with the Bourne shell executing our pipeline
            execl("/bin/sh", "sh", "-c", pipeline.c_str(), nullptr);

            // If execl returns, an irrecoverable kernel execution error occurred
            std::cerr << "[VIDEO FATAL] Failed to execute shell binary.\n";
            _exit(1);
        }
        else
        {
            // --- PARENT PROCESS CONTEXT ---
            m_child_pid = pid;
            std::cout << "[VIDEO] FPV Stream launched (PID: " << m_child_pid << ") targeting " << target_ip << ":" << target_port << "\n";
            return true;
        }
    }

    /**
     * @brief Transmits a POSIX termination signal to the entire Process Group.
     */
    void stop()
    {
        if (m_child_pid > 0)
        {
            std::cout << "[VIDEO] Terminating FPV Stream (Process Group: " << m_child_pid << ")...\n";

            // CRITICAL: Notice the negative sign (-m_child_pid).
            // This tells the OS to kill the entire Process Group, not just the shell wrapper.
            kill(-m_child_pid, SIGTERM);

            // Block temporarily (a few milliseconds) to reap the dead processes
            int status;
            waitpid(m_child_pid, &status, 0);

            m_child_pid = -1;
            std::cout << "[VIDEO] Pipeline successfully halted. Hardware freed.\n";
        }
    }

    /**
     * @brief Exposes the execution status of the pipeline.
     */
    bool is_running() const
    {
        return m_child_pid > 0;
    }

private:
    pid_t m_child_pid; ///< Operating System Process ID of the active execution group.
};