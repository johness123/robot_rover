#pragma once
#include <cstdint>

#pragma pack(push, 1) // Ép lề 1-byte chặt chẽ

struct UdpSecurityHeader
{
    uint32_t magic_key; // 0x524F5652 ("ROVR")
    uint32_t sequence_num;
};

enum class ManeuverType : uint8_t
{
    FRONT_WHEEL_STEER = 0,
    REAR_WHEEL_STEER = 1,
    CRAB_WALK = 2,
    POINT_TURN = 3
};

struct RoverIntentCommand
{
    UdpSecurityHeader header;

    uint8_t actor_id; // Luôn là 0x01
    uint8_t maneuver_type;
    int8_t throttle; // -100 đến 100
    int8_t steering; // -100 đến 100
    uint8_t emergency_brake;
};

#pragma pack(push, 1)
struct HardwareWheelCommand
{
    uint8_t actor_id; // 1 Byte -> 0x01

    // --- Vận tốc độc lập (-100 đến 100) ---
    int8_t power_fl; // Trước trái
    int8_t power_fr; // Trước phải
    int8_t power_ml; // Giữa trái
    int8_t power_mr; // Giữa phải
    int8_t power_rl; // Sau trái
    int8_t power_rr; // Sau phải

    // --- Góc lái servo (0 đến 180 độ, 90 là thẳng) ---
    uint8_t steer_fl;
    uint8_t steer_fr;
    uint8_t steer_ml;
    uint8_t steer_mr;
    uint8_t steer_rl;
    uint8_t steer_rr;

    uint8_t emergency_brake; // 1 Byte
};

// typedef HardwareWheelCommand WheelActorCommand;

struct RoverIncomingHandshake
{
    uint32_t handshake_key; ///< Predefined raw token pattern utilized as the authentication handshake passphrase.
    uint8_t reserved[8];    ///< Auxiliary padding allocated for structural stability and future telemetry allocation.
};

#pragma pack(pop)