#include "amr_core/kinematics.hpp"


void Kinematics::calculateRPM(double linear_x, double angular_z, double wheel_radius, double distance_wheels, double gear_ratio, int16_t& left_rpm, int16_t& right_rpm) {
    const double factor = 60.0 / (2.0 * M_PI * wheel_radius);
    left_rpm = static_cast<int16_t>(round(factor * (linear_x - (distance_wheels / 2.0) * angular_z) * gear_ratio));
    right_rpm = static_cast<int16_t>(round(factor * (linear_x + (distance_wheels / 2.0) * angular_z) * gear_ratio));
}

void Kinematics::calculateVW(int16_t left_rpm, int16_t right_rpm, double wheel_radius, double distance_wheels, double gear_ratio, double& v, double& w) {
    double real_motor_rpm_left = static_cast<double>(left_rpm) / gear_ratio;
    double real_motor_rpm_right = -static_cast<double>(right_rpm) / gear_ratio;
    v = (real_motor_rpm_left + real_motor_rpm_right) * (M_PI * wheel_radius / 60.0);
    w = (real_motor_rpm_right - real_motor_rpm_left) * (M_PI * wheel_radius / (30.0 * distance_wheels));
} 