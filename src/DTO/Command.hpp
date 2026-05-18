#ifndef COMMAND_DTOS_HPP
#define COMMAND_DTOS_HPP

#include <cstdint>

#pragma pack(push, 1)

/**
 * @struct WheelActorCommand
 * @brief DTO representing a decoupled payload for 6-wheel independent steering.
 *
 * This structure explicitly maps the actor ID and the 8 operational frames into
 * distinct 8-bit fields, avoiding network endianness issues and compiler padding overhead.
 */
struct WheelActorCommand
{
    uint8_t actor_id;
    uint8_t motor_power;
    uint8_t steer_fl;
    uint8_t steer_fr;
    uint8_t steer_ml;
    uint8_t steer_mr;
    uint8_t steer_rl;
    uint8_t steer_rr;
    uint8_t emergency_brake;
};

/**
 * @struct CameraMountCommand
 * @brief DTO for controlling a 2-axis (Pan-Tilt) camera gimbal mechanism.
 *
 * Uses 8-bit unsigned integers to represent absolute physical angles
 * (typically strictly bound between 0-180 degrees) for servo motors.
 */
struct CameraMountCommand
{
    uint8_t actor_id;
    uint8_t pan_angle;
    uint8_t tilt_angle;
};

#pragma pack(pop)

#endif // COMMAND_DTOS_HPP