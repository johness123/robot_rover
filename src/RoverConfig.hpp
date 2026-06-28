#pragma once
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

struct RoverConfig
{
    double track_width;
    double wheel_radius;
    double length_front;
    double length_rear;

    double max_steer_angle_rad;
    int8_t motor_deadzone;

    std::string camera_commander_port;

    // Servo offsets
    int8_t offset_fl, offset_fr, offset_ml, offset_mr, offset_rl, offset_rr;

    static RoverConfig load_from_file(const std::string &filepath)
    {
        RoverConfig config = {0};

        try
        {
            std::ifstream file(filepath);
            if (!file.is_open())
            {
                throw std::runtime_error("Không tìm thấy file " + filepath);
            }

            json j;
            file >> j;

            // Nạp kích thước
            config.track_width = j["dimensions"]["track_width_mm"];
            config.wheel_radius = j["dimensions"]["wheel_radius_mm"];
            config.length_front = j["dimensions"]["length_front_mm"];
            config.length_rear = j["dimensions"]["length_rear_mm"];

            // Nạp giới hạn cơ khí
            double max_steer_deg = j["limits"]["max_steer_angle_deg"];
            config.max_steer_angle_rad = max_steer_deg * (M_PI / 180.0);
            config.motor_deadzone = j["limits"]["motor_deadzone_percent"];

            // Nạp Servo Offset
            config.offset_fl = j["servo_offsets_deg"]["fl"];
            config.offset_fr = j["servo_offsets_deg"]["fr"];
            config.offset_ml = j["servo_offsets_deg"]["ml"];
            config.offset_mr = j["servo_offsets_deg"]["mr"];
            config.offset_rl = j["servo_offsets_deg"]["rl"];
            config.offset_rr = j["servo_offsets_deg"]["rr"];

            config.camera_commander_port = j["camera_commander_port"];

            std::cout << "[CONFIG] Đã nạp thành công file cấu hình cơ khí!\n";
        }
        catch (const std::exception &e)
        {
            std::cerr << "[LỖI CONFIG] Khởi tạo thất bại: " << e.what() << "\n";
            std::cerr << "--- HỆ THỐNG SẼ DÙNG THÔNG SỐ ZERO (NGUY HIỂM) ---\n";
        }

        return config;
    }
};