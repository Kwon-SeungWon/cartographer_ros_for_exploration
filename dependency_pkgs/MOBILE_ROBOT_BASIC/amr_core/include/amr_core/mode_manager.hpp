#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "amr_serial/srv/get_mode.hpp"

class ModeManager {
public:
    enum class State {
        INIT, AUTO, MANUAL, EMERGENCY, JOYSTICK
    };

    ModeManager(rclcpp::Node* node);
    void checkMode();
    State getState() const;
    void setManualFlag(bool flag);
    void setEmergencyFlag(bool flag);
private:
    rclcpp::Node* node_;
    State state_;
    bool manual_flag_ = false;
    bool emergency_flag_ = false;
    rclcpp::Client<amr_serial::srv::GetMode>::SharedPtr client_;
    void handle_get_mode_response(rclcpp::Client<amr_serial::srv::GetMode>::SharedFuture future);
}; 