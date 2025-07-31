#pragma once
#include <cmath>
#include <cstdint>

class Kinematics {
public:
    static void calculateRPM(double linear_x, 
                            double angular_z, 
                            double wheel_radius, 
                            double distance_wheels, 
                            double gear_ratio, 
                            int16_t& left_rpm, 
                            int16_t& right_rpm);

    static void calculateVW(int16_t left_rpm, 
                            int16_t right_rpm, 
                            double wheel_radius, 
                            double distance_wheels, 
                            double gear_ratio, 
                            double& v, 
                            double& w);
}; 