#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>
#include <cstring>
#include <unistd.h>
#include <can_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <queue>
#include <mutex>
#include <vector>
#include <memory>
#include <thread>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <unordered_map>
#include <functional>

#include "amr_core/velocity_profile.h"
#include "amr_core/ekf_odometry.h"
#include "amr_core/can.hpp"
#include "amr_core/kinematics.hpp"
#include "amr_core/mode_manager.hpp"

#define Math_PI 3.14159
#define SIGNIFICANT_FIGURES 100

namespace AMR {

enum class State : uint16_t {
    INIT = 0x00,
    IDLE = 0x10,
    AUTO = 0x20,
    MANUAL = 0x30,
    DOCKING = 0x40,
    UNDOCKING = 0x50,
    CHARGING = 0x60,
    MANIPULATION = 0x70,
    EMERGENCY = 0x80,
    STOP = 0x90,
    SLAM = 0x100
};

using CurrentState = State;
using LastState = State;

struct Motor {
    double enc_l;
    double enc_r;
};

struct Error {
    int right;
    int left;
};

struct RobotState {
    double x, y, z;
    double roll, pitch, yaw;
};

class Core : public rclcpp::Node {
public:
    Core();
    ~Core();
    void spin();
    geometry_msgs::msg::Twist parse();

protected:
    // --- ROS2 Interface ---
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctr_vel_pub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr enc_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr docking_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr enc_vel_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_machine_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr alarm_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_backup_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

    // --- State Machine ---
    std::map<State, std::shared_ptr<std::function<void()>>> state_machine;
    std::pair<CurrentState, LastState> state;
    State state_before;
    bool interface_active_;
    rclcpp::Time last_state_update_time_;

    // --- Parameters ---
    std::string nav_vel_parameter, enc_vel_parameter, ctr_vel_parameter, manual_vel_parameter, docking_vel_parameter, RobotName;
    std::string odom_parameter, odom_tf_parameter;
    double wheel_radius, distance_wheels, gear_ratio;
    int id_enc, id_rpm, mode, encoder_resolution, pole_number, max_rpm, break_option, hall_offset;
    double rated_current, max_current, loop_hz;
    uint32_t Main1, Main2, Sub, Upper_id, Lower_id, Motor_driver;
    double enc_vel_gain, enc_wvel_gain;
    geometry_msgs::msg::Twist acc;
    bool use_ekf;
    bool warnig_flag;
    bool running_;
    bool can_mode = false;

    // --- State Data ---
    std::shared_ptr<geometry_msgs::msg::Twist> ctr_vel, nav_vel, enc_vel, manual_vel, docking_vel, joy_vel;
    nav_msgs::msg::Odometry odom;
    geometry_msgs::msg::TransformStamped odom_tf;
    RobotState* robot_state;
    double dx, dy, dyaw, dt;
    double current_time, last_time;

    // --- Motor/Error ---
    Motor* motor;
    Error* error;

    // --- EKF ---
    std::unique_ptr<EKFOdometry> ekf_odometry;

    // --- Velocity Profile ---
    std::shared_ptr<VelocityProfile> profiler;

    // --- Module Members ---
    std::shared_ptr<CAN_> can_;
    std::shared_ptr<ModeManager> mode_manager_;

    // --- Threads ---
    boost::thread alarm_thread_;

    // --- Callback Functions ---
    virtual void navVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void encVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void manualCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void dockingVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void joyVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);

    
    void encCallback(const can_msgs::msg::Frame::SharedPtr msgs);
    void errorCallback(const can_msgs::msg::Frame::SharedPtr msgs);
    void hallCallback(const can_msgs::msg::Frame::SharedPtr msgs);
    void timerCallback();
    void alarmCallback();
    void SendState();
    void alarmThread(int rate);
    void SendEmergencyStop();
    void Init();
    void setParam();
    void getParam();
    void setPublisher();
    void setSubscriber();
    void setService();
    void settingMotor();
    void startMotor();
    void velocityMode();
    void synchronousMode();
    void asynchronousMode();
    void motorEnable();
    void save();
    virtual void stateTimerCallback();
    virtual void controlTimerCallback();
    void stateMachineCallback(const std_msgs::msg::String::SharedPtr msg);
    void stateBackupCallback();
    virtual void stateInit();
    virtual void stateIdle();
    virtual void stateAuto();
    virtual void stateManual();
    virtual void stateDocking();
    virtual void stateUndocking();
    virtual void stateCharging();
    virtual void stateManipulating();
    virtual void stateStop();
    virtual void stateEmergency();
    void updateBasicOdometry(double dt);
    void updateEKFOdometry(double dt);
    void publishOdometry();
    // --- CAN/Mode/Kinematics 연동 ---
    void canFrameCallback(const can_frame& frame);
    void runCanModeControl();
    void runLegacyControl();
};

} // namespace AMR 