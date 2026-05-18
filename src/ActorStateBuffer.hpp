#ifndef ACTOR_STATE_BUFFER_HPP
#define ACTOR_STATE_BUFFER_HPP

#include <mutex>
#include <optional>
#include <cstring>
#include <chrono>

/**
 * @class ActorStateBuffer
 * @brief A thread-safe buffer with temporal tracking for actor state management.
 *
 * Encapsulates a generic DTO and provides a mutex-protected environment for
 * concurrent read/write operations. Incorporates a timestamping mechanism to
 * facilitate network failsafe implementations.
 *
 * @tparam T The Data Transfer Object (DTO) structure type strictly aligned to 1-byte boundaries.
 */
template <typename T>
class ActorStateBuffer
{
public:
    ActorStateBuffer() : m_last_update(std::chrono::steady_clock::now()) {}

    /**
     * @brief Deserializes a raw byte array into the DTO and updates the temporal state.
     *
     * @param raw_data Pointer to the incoming byte array.
     * @param size Exact size of the byte array received.
     */
    void update_from_network(const uint8_t *raw_data, std::size_t size)
    {
        if (size != sizeof(T))
        {
            return;
        }

        T new_state;
        std::memcpy(&new_state, raw_data, sizeof(T));

        std::lock_guard<std::mutex> lock(m_mutex);
        m_state = new_state;
        m_last_update = std::chrono::steady_clock::now();
    }

    /**
     * @brief Consumes the current state for hardware execution.
     *
     * @return std::optional<T> Containing the state if available, or std::nullopt if empty.
     */
    std::optional<T> consume()
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_state.has_value())
        {
            T state = m_state.value();
            m_state.reset();
            return state;
        }
        return std::nullopt;
    }

    /**
     * @brief Evaluates whether the buffer's contents have exceeded a specified lifespan.
     *
     * @param timeout The maximum allowable duration since the last network update.
     * @return true If the elapsed time exceeds the timeout threshold.
     * @return false If the buffer was updated recently.
     */
    bool is_stale(std::chrono::milliseconds timeout) const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto now = std::chrono::steady_clock::now();
        return (now - m_last_update) > timeout;
    }

private:
    mutable std::mutex m_mutex;
    std::optional<T> m_state;
    std::chrono::steady_clock::time_point m_last_update;
};

#endif // ACTOR_STATE_BUFFER_HPP