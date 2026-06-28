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
        case ManeuverType::CRAB_WALK:
            solve_crab_walk(throttle_norm, steering_norm, hw_cmd);
            break;
        case ManeuverType::POINT_TURN:
            solve_point_turn(throttle_norm, steering_norm, hw_cmd);
            break;

        default:
            set_all_straight(hw_cmd);
            set_all_power(hw_cmd, intent.throttle);
            break;
        }

        return hw_cmd;
    }

private:
    RoverConfig m_config;

    /**
     * @brief Executes true omnidirectional translational movement (Swerve / Crab Walk).
     * Incorporates 2D vector calculation from both Throttle and Steering.
     * Enforces strict [0, 180] degree servo bounds via motor polarity reversal.
     */
    void solve_crab_walk(double throttle_norm, double steering_norm, HardwareWheelCommand &hw) const
    {
        // Khử nhiễu (Deadzone) tay cầm: Nếu không có ai đụng vào Joystick, xe phải đứng im
        if (std::abs(throttle_norm) < 0.05 && std::abs(steering_norm) < 0.05)
        {
            set_all_straight(hw);
            set_all_power(hw, 0);
            return;
        }

        // --- 1. TÍNH TOÁN VECTOR ĐA HƯỚNG (2D Translation Vector) ---
        // Biến X và Y của tay cầm thành Góc (Radian) và Độ lớn tốc độ (%)
        // Dùng atan2(X, Y) thay vì atan2(Y, X) để quy ước: Tiến thẳng Y là 0 độ.
        // double target_angle = std::atan2(steering_norm, throttle_norm);
        // double target_speed = std::hypot(steering_norm, throttle_norm) * 100.0;
        // double target_angle = steering_norm * 1.8;
        // double target_speed = throttle_norm;

        // // Khóa tốc độ không cho vượt quá 100% khi đẩy chéo Joystick
        // target_speed = std::min(target_speed, 100.0);

        // // --- 2. XỬ LÝ GIỚI HẠN SERVO BẰNG CÁCH ĐẢO CHIỀU ĐỘNG CƠ ---
        // // Hàm atan2 sẽ trả về góc từ -180 đến 180 độ (-Pi đến Pi).
        // // Nếu góc lớn hơn 90 độ hoặc nhỏ hơn -90 độ, xe đang muốn đi lùi chéo.
        // // Để không làm gãy Servo, ta giữ Servo hướng về nửa bán cầu trước, và đảo chiều quay của bánh.
        // if (target_angle > (M_PI / 2.0))
        // {
        //     target_angle -= M_PI;         // Bật ngược góc lại 180 độ
        //     target_speed = -target_speed; // Đảo điện động cơ thành lùi
        // }
        // else if (target_angle < -(M_PI / 2.0))
        // {
        //     target_angle += M_PI;         // Bật ngược góc lại 180 độ
        //     target_speed = -target_speed; // Đảo điện động cơ thành lùi
        // }

        // // --- 3. BẢO VỆ CƠ KHÍ KHUNG GẦM ---
        // // Dù Servo quay được 0-180, nhưng càng lái (Linkage) có thể sẽ bị cấn vào khung xe
        // // nếu bẻ quá gắt. Ta phải khóa góc trong giới hạn m_config.max_steer_angle_rad
        // target_angle = std::clamp(target_angle, -m_config.max_steer_angle_rad, m_config.max_steer_angle_rad);
        double target_angle, target_speed;
        if (std::abs(steering_norm) <= 0.5)
        {
            target_angle = steering_norm * 180;
            target_speed = throttle_norm * 100;
        }
        else if (steering_norm > 0.5)
        {
            target_angle = (steering_norm * 180) - 180;
            target_speed = -throttle_norm * 100;
        }
        else
        {
            target_angle = (steering_norm * 180) + 180;
            target_speed = -throttle_norm * 100;
        }

        // --- 4. ÁP DỤNG XUỐNG PHẦN CỨNG ---
        // Toàn bộ 6 bánh bẻ song song cùng một góc (Đã được cộng bù trừ Offset an toàn)
        hw.steer_fl = deg_to_servo(target_angle, m_config.offset_fl);
        hw.steer_fr = deg_to_servo(target_angle, m_config.offset_fr);
        hw.steer_ml = deg_to_servo(target_angle, m_config.offset_ml);
        hw.steer_mr = deg_to_servo(target_angle, m_config.offset_mr);
        hw.steer_rl = deg_to_servo(target_angle, m_config.offset_rl);
        hw.steer_rr = deg_to_servo(target_angle, m_config.offset_rr);

        // Toàn bộ 6 bánh nhận cùng một lực quay (Không có vi sai)
        int8_t final_power = clamp_power(target_speed);
        set_all_power(hw, final_power);
    }

    /**
     * @brief Calculate
     */
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

        // 1. Tính góc (Radian) - ĐÃ SỬA LẠI DẤU: Trái (+), Phải (-)
        double angle_fl = std::atan(m_config.length_front / (R_icc + (m_config.track_width / 2.0)));
        double angle_fr = std::atan(m_config.length_front / (R_icc - (m_config.track_width / 2.0)));

        // Áp dụng thuật toán quy đổi sang Servo + Cộng Offset rập khuôn từ File JSON
        hw.steer_fl = rad_to_servo(angle_fl, m_config.offset_fl);
        hw.steer_fr = rad_to_servo(angle_fr, m_config.offset_fr);
        hw.steer_ml = rad_to_servo(0.0, m_config.offset_ml);
        hw.steer_mr = rad_to_servo(0.0, m_config.offset_mr);
        hw.steer_rl = rad_to_servo(0.0, m_config.offset_rl);
        hw.steer_rr = rad_to_servo(0.0, m_config.offset_rr);

        // 2. Tính vi sai tốc độ bánh xe - ĐÃ SỬA LẠI DẤU: Trái (+), Phải (-)
        double R_fl = std::hypot(m_config.length_front, R_icc + (m_config.track_width / 2.0));
        double R_fr = std::hypot(m_config.length_front, R_icc - (m_config.track_width / 2.0));
        double R_ml = std::abs(R_icc + (m_config.track_width / 2.0));
        double R_mr = std::abs(R_icc - (m_config.track_width / 2.0));
        double R_rl = std::hypot(m_config.length_rear, R_icc + (m_config.track_width / 2.0));
        double R_rr = std::hypot(m_config.length_rear, R_icc - (m_config.track_width / 2.0));

        // 3. Áp dụng lực ga - BỌC abs(R_icc) ĐỂ TRÁNH LỖI ĐẢO CHIỀU GA
        double abs_R_icc = std::abs(R_icc);

        hw.power_fl = clamp_power(throttle_norm * (R_fl / abs_R_icc) * 100);
        hw.power_fr = clamp_power(throttle_norm * (R_fr / abs_R_icc) * 100);
        hw.power_ml = clamp_power(throttle_norm * (R_ml / abs_R_icc) * 100);
        hw.power_mr = clamp_power(throttle_norm * (R_mr / abs_R_icc) * 100);
        hw.power_rl = clamp_power(throttle_norm * (R_rl / abs_R_icc) * 100);
        hw.power_rr = clamp_power(throttle_norm * (R_rr / abs_R_icc) * 100);
    }

    /**
     * @brief Executes zero-radius pivot (Point Turn / Tank Turn).
     * The rover rotates around its exact geometric center.
     * Incorporates hardware constraints: Servos strictly operate within [0, 180] degrees.
     */
    void solve_point_turn(double throttle_norm, double steering_norm, HardwareWheelCommand &hw) const
    {
        // Chế độ xoay tại chỗ chỉ dùng trục X (steering) để xoay trái/phải.
        if (std::abs(steering_norm) < 0.05)
        {
            set_all_straight(hw);
            set_all_power(hw, 0);
            return;
        }

        double half_track = m_config.track_width / 2.0;

        // --- 1. TÍNH GÓC GÌN GIỮ GIỚI HẠN SERVO (0 ĐẾN 180 ĐỘ) ---
        // Hàm atan(Length / Width) luôn trả về góc từ 0 đến 90 độ.
        // Do đó, Servo sẽ chỉ dao động trong khoảng (90 - góc) đến (90 + góc),
        // TUYỆT ĐỐI AN TOÀN, không bao giờ đâm thủng trần 180 hay đáy 0 của phần cứng.
        double angle_front = std::atan(m_config.length_front / half_track);
        double angle_rear = std::atan(m_config.length_rear / half_track);

        // Tạo đội hình chữ X (Mũi bánh trước chụm vào, mũi bánh sau tõe ra)
        hw.steer_fl = rad_to_servo(angle_front, m_config.offset_fl);  // > 90 độ (Bẻ phải)
        hw.steer_fr = rad_to_servo(-angle_front, m_config.offset_fr); // < 90 độ (Bẻ trái)
        hw.steer_ml = rad_to_servo(0.0, m_config.offset_ml);          // 90 độ (Giữ thẳng)
        hw.steer_mr = rad_to_servo(0.0, m_config.offset_mr);          // 90 độ (Giữ thẳng)
        hw.steer_rl = rad_to_servo(-angle_rear, m_config.offset_rl);  // < 90 độ (Bẻ trái)
        hw.steer_rr = rad_to_servo(angle_rear, m_config.offset_rr);   // > 90 độ (Bẻ phải)

        // --- 2. TÍNH BÁN KÍNH ĐỂ CÂN BẰNG TỐC ĐỘ ---
        double R_front = std::hypot(m_config.length_front, half_track);
        double R_mid = half_track;
        double R_rear = std::hypot(m_config.length_rear, half_track);

        // Tìm bán kính lớn nhất để chuẩn hóa lực ga, tránh motor bị quá dòng
        double R_max = std::max(R_front, R_rear);

        // --- 3. ĐẢO CỰC ĐỘNG CƠ (BÍ QUYẾT TRÁNH QUAY SERVO 180 ĐỘ) ---
        double rot_power = steering_norm * 100.0;

        // Nếu xoay sang phải (rot_power > 0): Dàn bánh bên TRÁI phải tiến lên (số Dương)
        hw.power_fl = clamp_power(rot_power * (R_front / R_max));
        hw.power_ml = clamp_power(rot_power * (R_mid / R_max));
        hw.power_rl = clamp_power(rot_power * (R_rear / R_max));

        // Nếu xoay sang phải (rot_power > 0): Dàn bánh bên PHẢI phải lùi lại (gắn dấu Trừ)
        // Nhờ việc lùi lại này, Servo FR và RR không cần phải quay ngược ra đằng sau!
        hw.power_fr = clamp_power(-rot_power * (R_front / R_max));
        hw.power_mr = clamp_power(-rot_power * (R_mid / R_max));
        hw.power_rr = clamp_power(-rot_power * (R_rear / R_max));
    }

    uint8_t rad_to_servo(double rad, int8_t offset) const
    {
        double deg = rad * (180.0 / M_PI);
        // Đưa góc chuẩn về 90, sau đó cộng bù trừ (ví dụ lệch trái/phải)
        double final_deg = deg + 90.0 + offset;
        return static_cast<uint8_t>(std::clamp(final_deg, 0.0, 180.0));
    }

    // Due to servos' install orientation, the 0 degree is left side, so final degree has to be added with 90
    uint8_t deg_to_servo(double deg, int8_t offset) const
    {
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