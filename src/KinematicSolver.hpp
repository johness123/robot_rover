#ifndef KINEMATIC_SOLVER_HPP
#define KINEMATIC_SOLVER_HPP

#include <cmath>
#include <algorithm>
#include "DTO/Command.hpp"
#include "RoverConfig.hpp"

class KinematicSolver
{
public:
    // Khởi tạo Solver với cấu hình đọc từ JSON
    explicit KinematicSolver(const RoverConfig &config) : m_config(config) {}

    HardwareWheelCommand solve(const RoverIntentCommand &intent) const
    {
        HardwareWheelCommand hw_cmd = {0};
        hw_cmd.actor_id = 0x01;
        hw_cmd.emergency_brake = intent.emergency_brake;

        if (intent.emergency_brake == 1)
        {
            set_all_straight(hw_cmd);
            set_all_power(hw_cmd, 0);
            return hw_cmd;
        }

        double throttle_norm = intent.throttle / 100.0;
        double steering_norm = intent.steering / 100.0;

        switch (static_cast<ManeuverType>(intent.maneuver_type))
        {
        case ManeuverType::FRONT_WHEEL_STEER:
            solve_ackermann(throttle_norm, steering_norm, hw_cmd);
            break;
        // Gọi hàm cho Crab Walk, Point Turn ở đây...
        default:
            set_all_straight(hw_cmd);
            set_all_power(hw_cmd, intent.throttle);
            break;
        }

        return hw_cmd;
    }

private:
    RoverConfig m_config;

    void solve_ackermann(double throttle_norm, double steering_norm, HardwareWheelCommand &hw) const
    {
        if (std::abs(steering_norm) < 0.05)
        {
            set_all_straight(hw);
            set_all_power(hw, static_cast<int8_t>(throttle_norm * 100));
            return;
        }

        double virtual_angle = steering_norm * m_config.max_steer_angle_rad;
        double R_icc = m_config.length_front / std::tan(virtual_angle);

        // Tính góc (Radian)
        double angle_fl = std::atan2(m_config.length_front, R_icc - (m_config.track_width / 2.0));
        double angle_fr = std::atan2(m_config.length_front, R_icc + (m_config.track_width / 2.0));

        // Áp dụng thuật toán quy đổi sang Servo + Cộng Offset rập khuôn từ File JSON
        hw.steer_fl = rad_to_servo(angle_fl, m_config.offset_fl);
        hw.steer_fr = rad_to_servo(angle_fr, m_config.offset_fr);
        hw.steer_ml = rad_to_servo(0.0, m_config.offset_ml);
        hw.steer_mr = rad_to_servo(0.0, m_config.offset_mr);
        hw.steer_rl = rad_to_servo(0.0, m_config.offset_rl);
        hw.steer_rr = rad_to_servo(0.0, m_config.offset_rr);

        // Tính vi sai tốc độ bánh xe
        double R_fl = std::hypot(m_config.length_front, R_icc - (m_config.track_width / 2.0));
        double R_fr = std::hypot(m_config.length_front, R_icc + (m_config.track_width / 2.0));
        double R_ml = std::abs(R_icc - (m_config.track_width / 2.0));
        double R_mr = std::abs(R_icc + (m_config.track_width / 2.0));
        double R_rl = std::hypot(m_config.length_rear, R_icc - (m_config.track_width / 2.0));
        double R_rr = std::hypot(m_config.length_rear, R_icc + (m_config.track_width / 2.0));

        // Áp dụng lực ga có tính Deadzone
        hw.power_fl = clamp_power(throttle_norm * (R_fl / R_icc) * 100);
        hw.power_fr = clamp_power(throttle_norm * (R_fr / R_icc) * 100);
        hw.power_ml = clamp_power(throttle_norm * (R_ml / R_icc) * 100);
        hw.power_mr = clamp_power(throttle_norm * (R_mr / R_icc) * 100);
        hw.power_rl = clamp_power(throttle_norm * (R_rl / R_icc) * 100);
        hw.power_rr = clamp_power(throttle_norm * (R_rr / R_icc) * 100);
    }

    uint8_t rad_to_servo(double rad, int8_t offset) const
    {
        double deg = rad * (180.0 / M_PI);
        // Đưa góc chuẩn về 90, sau đó cộng bù trừ (ví dụ lệch trái/phải)
        double final_deg = deg + 90.0 + offset;
        return static_cast<uint8_t>(std::clamp(final_deg, 0.0, 180.0));
    }

    int8_t clamp_power(double power) const
    {
        // Nếu điện áp cấp quá yếu (dưới deadzone), motor không chạy được -> Trả về 0 để đỡ tốn pin
        if (std::abs(power) > 0 && std::abs(power) < m_config.motor_deadzone)
        {
            return 0;
        }
        return static_cast<int8_t>(std::clamp(power, -100.0, 100.0));
    }

    void set_all_straight(HardwareWheelCommand &hw) const
    {
        hw.steer_fl = rad_to_servo(0.0, m_config.offset_fl);
        hw.steer_fr = rad_to_servo(0.0, m_config.offset_fr);
        hw.steer_ml = rad_to_servo(0.0, m_config.offset_ml);
        hw.steer_mr = rad_to_servo(0.0, m_config.offset_mr);
        hw.steer_rl = rad_to_servo(0.0, m_config.offset_rl);
        hw.steer_rr = rad_to_servo(0.0, m_config.offset_rr);
    }

    void set_all_power(HardwareWheelCommand &hw, int8_t p) const
    {
        hw.power_fl = hw.power_fr = hw.power_ml = hw.power_mr = hw.power_rl = hw.power_rr = p;
    }
};

#endif