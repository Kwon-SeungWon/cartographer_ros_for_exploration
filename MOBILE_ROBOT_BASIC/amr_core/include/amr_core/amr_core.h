#ifndef __AMR_CORE_H__
#define __AMR_CORE_H__

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

// For state machine messages from the Interface node.
#include "san_msgs/msg/state_machine.hpp"

#define Math_PI 3.14159
#define SIGNIFICANT_FIGURES 100

namespace AMR
{

enum class State : uint16_t {
    INIT            = 0x00, ///< 00 Initialization state
    IDLE            = 0x10, ///< 16 Idle state
    AUTO            = 0x20, ///< 32 Autonomous operation state
    MANUAL          = 0x30, ///< 48 Manual control state
    DOCKING         = 0x40, ///< 64 Docking procedure state
    UNDOCKING       = 0x50, ///< 80 Undocking procedure state
    CHARGING        = 0x60, ///< 96 Charging state
    MANIPULATION    = 0x70, ///< 112 Work execution state
    EMERGENCY       = 0x80, ///< 128 Emergency state
    STOP            = 0x90, ///< 144 Stop state
    SLAM            = 0x100 ///< 160 SLAM state
};

using CurrentState = State;
using LastState = State;

typedef struct 
{
    double enc_l;
    double enc_r;
} Motor;

typedef struct 
{
    int right;
    int left;
} Error;

struct RobotState
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

class Core : public rclcpp::Node
{
private:
    // Publishers, Subscribers, and Timers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctr_vel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr enc_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr docking_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_machine_sub_; // Added for Interface state updates

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr alarm_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_backup_timer_; // Backup timer for state updates

    rclcpp::Time current_time_;
    rclcpp::Time last_time_;
    rclcpp::Time last_state_update_time_; // Last time a state update was received

    std::shared_ptr<geometry_msgs::msg::Twist> ctr_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> nav_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> enc_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> manual_vel;
    std::shared_ptr<geometry_msgs::msg::Twist> docking_vel;

    boost::thread alarm_thread_;

    ////////////parameter////////////
    std::string nav_vel_parameter;
    std::string enc_vel_parameter;
    std::string ctr_vel_parameter;
    std::string manual_vel_parameter;
    std::string docking_vel_parameter;

    std::string odom_parameter;
    std::string odom_tf_parameter;

    ////////////odometry param///////////
    double current_time, last_time;
    double enc_vel_gain, enc_wvel_gain;
    double dx, dy, dyaw, dt;
    RobotState *robot_state;
    nav_msgs::msg::Odometry odom;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    geometry_msgs::msg::TransformStamped odom_tf;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
    

    bool running_;
    bool warnig_flag;
    double wheel_radius;
    double distance_wheels;
    double gear_ratio;
    int id_enc;
    int id_rpm;
    int mode;
    int encoder_resolution;
    int pole_number;
    int max_rpm;
    int break_option;
    double rated_current;
    double max_current;
    int hall_offset;
    geometry_msgs::msg::Twist acc;
    double loop_hz;

    int32_t Main1;
    int32_t Main2;
    int32_t Sub;
    int32_t Motor_driver;
    
    Motor* motor;
    Error* error;

    // Additional variables for state machine management
    std::map<State, std::shared_ptr<std::function<void()>>> state_machine;
    std::pair<CurrentState, LastState> state;
    State state_before;
    
    // New flag to track if Interface state updates are active.
    bool interface_active_;

    // Callback wrapper functions
    virtual void navVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void encVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);
    virtual void manualCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);    
    virtual void dockingVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs);    

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

    // Timer callbacks for state machine
    virtual void stateTimerCallback();
    virtual void controlTimerCallback();

    // New callback for state machine messages from Interface node.
    void stateMachineCallback(const std_msgs::msg::String::SharedPtr msg);
    // Backup callback if no state updates are received from Interface node.
    void stateBackupCallback();

    std::shared_ptr<VelocityProfile> profiler;
    
    // State machine functions
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

    // Parameters
    bool use_ekf;
    std::unique_ptr<EKFOdometry> ekf_odometry;

    void updateBasicOdometry(double dt);
    void updateEKFOdometry(double dt);
    void publishOdometry();

public:
    Core();
    ~Core();
    
    void spin();

    geometry_msgs::msg::Twist parse();
};

}  // namespace Redbot

#endif  // __REDBOT_CORE_H__
