#ifndef __TELEOP_KEYBOARD_HPP__
#define __TELEOP_KEYBOARD_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/int8.hpp>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <unordered_map>
#include <memory>
#include <vector>
#include <sys/select.h>

enum class State
{
    MANUAL,
    AUTO
};

// Map for movement and speed bindings
extern std::unordered_map<char, std::vector<float>> moveBindings;
extern std::unordered_map<char, std::vector<float>> speedBindings;
extern std::unordered_map<char, std::vector<float>> utilBindings;

class TeleopKeyboard : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr util_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr auto_run_client_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;

    std::string topic_name_;
    std::string util_name_;
    std::string auto_run_service_name_;
    double speed_;
    double turn_;
    double x_, y_, z_, th_;
    char key_;
    double control_hz_;
    State state_;
    geometry_msgs::msg::Twist twist_;

    void getParam();
    int getch(); 

public:
    TeleopKeyboard();
    ~TeleopKeyboard() = default;
    void timer1_callback();
    void timer2_callback();
};

#endif // __TELEOP_KEYBOARD_HPP__
