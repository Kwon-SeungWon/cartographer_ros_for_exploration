#include "amr_core/amr_core.hpp"
#include "amr_core/can.hpp"
#include "amr_core/kinematics.hpp"
#include "amr_core/mode_manager.hpp"

using namespace std::chrono_literals;

namespace AMR {

//---------------------------------------------------------------------
// Core Class Implementation
//---------------------------------------------------------------------

Core::Core() : Node("amr_core"), interface_active_(false), can_mode(false)
{
    setParam();
    getParam();
    setPublisher();
    setSubscriber();
    Init();

    last_time = this->now().seconds();
}

Core::~Core()
{
    running_ = false;
}

void Core::Init()
{
    warnig_flag = false;

    // Reset and allocate velocity messages
    ctr_vel = std::make_shared<geometry_msgs::msg::Twist>();
    nav_vel = std::make_shared<geometry_msgs::msg::Twist>();
    enc_vel = std::make_shared<geometry_msgs::msg::Twist>();
    manual_vel = std::make_shared<geometry_msgs::msg::Twist>();
    docking_vel = std::make_shared<geometry_msgs::msg::Twist>();
    joy_vel = std::make_shared<geometry_msgs::msg::Twist>();

    running_ = true;

    // Velocity profiler initialization
    profiler = std::make_shared<VelocityProfile>(acc, loop_hz);

    // Init State machine timer (legacy backup timer)
    state_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/loop_hz), std::bind(&Core::stateTimerCallback, this));

    control_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/loop_hz), std::bind(&Core::controlTimerCallback, this));

    // --- CAN 관련 초기화는 can_mode가 true일 때만 ---
    if (can_mode) {
        can_ = std::make_shared<CAN_>();
        mode_manager_ = std::make_shared<ModeManager>(this);
        can_->setReceiveCallback([this](const can_frame& frame) {
            this->canFrameCallback(frame);
        });
    }

    // ToDoLists: 추후 Interface에 대한 State도 추가해야함
    // Initialize state machine with state callback functions using std::make_pair.
    state_machine.insert(std::make_pair(State::INIT, std::make_shared<std::function<void()>>(std::bind(&Core::stateInit, this))));
    state_machine.insert(std::make_pair(State::IDLE, std::make_shared<std::function<void()>>(std::bind(&Core::stateIdle, this))));
    state_machine.insert(std::make_pair(State::AUTO, std::make_shared<std::function<void()>>(std::bind(&Core::stateAuto, this))));
    state_machine.insert(std::make_pair(State::MANUAL, std::make_shared<std::function<void()>>(std::bind(&Core::stateManual, this))));
    state_machine.insert(std::make_pair(State::SLAM, std::make_shared<std::function<void()>>(std::bind(&Core::stateManual, this))));
    state_machine.insert(std::make_pair(State::DOCKING, std::make_shared<std::function<void()>>(std::bind(&Core::stateDocking, this))));
    state_machine.insert(std::make_pair(State::UNDOCKING, std::make_shared<std::function<void()>>(std::bind(&Core::stateUndocking, this))));
    state_machine.insert(std::make_pair(State::CHARGING, std::make_shared<std::function<void()>>(std::bind(&Core::stateCharging, this))));
    state_machine.insert(std::make_pair(State::MANIPULATION, std::make_shared<std::function<void()>>(std::bind(&Core::stateManipulating, this))));
    state_machine.insert(std::make_pair(State::STOP, std::make_shared<std::function<void()>>(std::bind(&Core::stateStop, this))));
    state_machine.insert(std::make_pair(State::EMERGENCY, std::make_shared<std::function<void()>>(std::bind(&Core::stateEmergency, this))));

    state = std::make_pair(State::INIT, State::INIT);
    state_before = State::INIT;

    // Backup Timer: if no /state_machine messages are received, fallback to internal state machine.
    last_state_update_time_ = this->now();
    state_backup_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Core::stateBackupCallback, this));

    odom_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom.header.frame_id = odom_parameter;
    odom.child_frame_id = odom_tf_parameter;

    odom_tf.header.frame_id = odom_parameter;
    odom_tf.child_frame_id = odom_tf_parameter;

    if (use_ekf) {
        ekf_odometry = std::make_unique<EKFOdometry>();
    }
}

void Core::setParam()
{
    this->declare_parameter("wheel_radius", 0.3);
    this->declare_parameter("distance_wheels", 0.63);
    this->declare_parameter("gear_ratio", 28.0);
    this->declare_parameter("id_rpm", 1);
    this->declare_parameter("id_enc", 1);
    this->declare_parameter("mode", 1);
    this->declare_parameter("encoder_resolution", 4096);
    this->declare_parameter("pole_number", 4);
    this->declare_parameter("max_rpm", 200);
    this->declare_parameter("break_option", 1);
    this->declare_parameter("rated_current", 8.0);
    this->declare_parameter("max_current", 16.0);
    this->declare_parameter("hall_offset", 240);
    this->declare_parameter("Main1", 0x1A1B101);
    this->declare_parameter("Sub", 0x1B1A101);
    this->declare_parameter("enc_vel_gain", 0.9);
    this->declare_parameter("enc_wvel_gain", 0.93);
    this->declare_parameter("vx_acc", 0.7);
    this->declare_parameter("vy_acc", 0.7);
    this->declare_parameter("w_acc", 0.7);
    this->declare_parameter("loop_hz", 20.0);
    this->declare_parameter("use_ekf", true);
    /////////////////////////////////////////////////////
    this->declare_parameter("nav_vel_parameter", "nav_vel");
    this->declare_parameter("enc_vel_parameter", "enc_vel");
    this->declare_parameter("manual_vel_parameter", "manual_vel");
    this->declare_parameter("docking_vel_parameter", "docking_vel");

    this->declare_parameter("ctr_vel_parameter", "cmd_vel");
    this->declare_parameter("odom_parameter", "odom");
    this->declare_parameter("odom_tf_parameter", "base_footprint");
    this->declare_parameter("can_mode", false);
    this->declare_parameter("Motor_driver", 0xB8B701);


    motor = new Motor{};
    error = new Error{};
    robot_state = new RobotState{};
}

void Core::getParam()
{
    this->get_parameter("wheel_radius", wheel_radius);
    this->get_parameter("distance_wheels", distance_wheels);
    this->get_parameter("gear_ratio", gear_ratio);
    this->get_parameter("id_rpm", id_rpm);
    this->get_parameter("id_enc", id_enc);
    this->get_parameter("mode", mode);
    this->get_parameter("encoder_resolution", encoder_resolution);
    this->get_parameter("pole_number", pole_number);
    this->get_parameter("max_rpm", max_rpm);
    this->get_parameter("break_option", break_option);
    this->get_parameter("rated_current", rated_current);
    this->get_parameter("max_current", max_current);
    this->get_parameter("hall_offset", hall_offset);
    this->get_parameter("Main1", Main1);
    this->get_parameter("Sub", Sub);
    this->get_parameter("enc_vel_gain", enc_vel_gain);
    this->get_parameter("enc_wvel_gain", enc_wvel_gain);
    this->get_parameter("vx_acc", acc.linear.x);
    this->get_parameter("vy_acc", acc.linear.y);
    this->get_parameter("w_acc", acc.angular.z);
    this->get_parameter("loop_hz", loop_hz);
    this->get_parameter("use_ekf", use_ekf);
    ///////////////////////////////////////////////////////////
    this->get_parameter("nav_vel_parameter", nav_vel_parameter);
    this->get_parameter("enc_vel_parameter", enc_vel_parameter);
    this->get_parameter("manual_vel_parameter", manual_vel_parameter);
    this->get_parameter("docking_vel_parameter", docking_vel_parameter);

    this->get_parameter("ctr_vel_parameter", ctr_vel_parameter);
    this->get_parameter("odom_parameter", odom_parameter);
    this->get_parameter("odom_tf_parameter", odom_tf_parameter);

    this->get_parameter("can_mode", can_mode);
    this->get_parameter("Motor_driver", Motor_driver);
    
}

void Core::setPublisher()
{
    ctr_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(ctr_vel_parameter, 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_parameter, 10);
}

void Core::setSubscriber()
{
    nav_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(nav_vel_parameter, 1, std::bind(&Core::navVelCallback, this, std::placeholders::_1));
    manual_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(manual_vel_parameter, 1, std::bind(&Core::manualCallback, this, std::placeholders::_1));
    docking_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(docking_vel_parameter, 1, std::bind(&Core::dockingVelCallback, this, std::placeholders::_1));
    joy_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("joy_vel", 1, std::bind(&Core::joyVelCallback, this, std::placeholders::_1));

    enc_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(enc_vel_parameter, 1, std::bind(&Core::encVelCallback, this, std::placeholders::_1));
    state_machine_sub_ = this->create_subscription<std_msgs::msg::String>("state_machine", 10, std::bind(&Core::stateMachineCallback, this, std::placeholders::_1));
}

void Core::controlTimerCallback()
{
    if (can_mode) {
        runCanModeControl();
    } else {
        runLegacyControl();
    }
}

void Core::runCanModeControl()
{

    if (mode_manager_) mode_manager_->checkMode();
    
    int16_t left_rpm = 0, right_rpm = 0;
    Kinematics::calculateRPM(ctr_vel->linear.x, ctr_vel->angular.z, wheel_radius, distance_wheels, gear_ratio, left_rpm, right_rpm);
   

    if (can_) {
        uint8_t data[8];
        int index = 0;

        data[index++] = static_cast<uint8_t>(PID_PNT_VEL_CMD);
        data[index++] = 1;
        data[index++] = left_rpm & 0xff;
        data[index++] = (left_rpm >> 8) & 0xff;
        data[index++] = 1;
        data[index++] = right_rpm & 0xff;
        data[index++] = (right_rpm >> 8) & 0xff;
        data[index++] = 1;
        
        bool send_result = can_->sendCan(DRIVER_ID, data);
        
        if (!send_result) {
            RCLCPP_ERROR(this->get_logger(), "[CAN] Failed to send CAN message");
        } else {
            // RCLCPP_INFO(this->get_logger(), "[CAN] Sent CAN message: left_rpm=%d, right_rpm=%d", left_rpm, right_rpm);
        }
    }
}

void Core::runLegacyControl()
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = ctr_vel->linear.x;
    msg.angular.z = ctr_vel->angular.z;
    ctr_vel_pub_->publish(msg);
}

void Core::navVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    *nav_vel = *msgs;
    if (state.first != State::STOP 
        && state.first != State::EMERGENCY
        && !interface_active_) {
            
        state.first = State::AUTO;

    }
}

void Core::dockingVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    *docking_vel = *msgs;
    if (state.first != State::STOP 
        && state.first != State::EMERGENCY
        && !interface_active_) {
            
        state.first = State::DOCKING;

    }
}

void Core::manualCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    // RCLCPP_INFO(this->get_logger(), "[MANUAL] Received velocity: linear.x=%.3f, angular.z=%.3f", msgs->linear.x, msgs->angular.z);
    if (warnig_flag)
        *manual_vel = profiler->reduceVelocity(*msgs);
    else
        *manual_vel = *msgs;
    
    if (state.first != State::STOP 
        && state.first != State::EMERGENCY
        && !interface_active_) {
            
        state.first = State::MANUAL;

    }
}

void Core::joyVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    *joy_vel = *msgs;

    if (state.first != State::STOP 
        && state.first != State::EMERGENCY) {
        
        if(mode_manager_->getState() == ModeManager::State::JOYSTICK) {
            
            state.first = State::MANUAL;
        
        } 
    }
}

void Core::updateBasicOdometry(double dt)
{
    dx = (enc_vel->linear.x * cos(robot_state->yaw) - enc_vel->linear.y * sin(robot_state->yaw)) * dt;
    dy = (enc_vel->linear.x * sin(robot_state->yaw) + enc_vel->linear.y * cos(robot_state->yaw)) * dt;
    dyaw = enc_vel->angular.z * dt;

    robot_state->x += dx;
    robot_state->y += dy;
    robot_state->yaw += dyaw;
}

void Core::updateEKFOdometry(double dt)
{
    ekf_odometry->setVelocity(enc_vel->linear.x, enc_vel->angular.z);
    ekf_odometry->predict(dt);
    ekf_odometry->update(robot_state->x, robot_state->y, robot_state->yaw);
    
    robot_state->x = ekf_odometry->getX();
    robot_state->y = ekf_odometry->getY();
    robot_state->yaw = ekf_odometry->getYaw();
}

void Core::publishOdometry()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_state->yaw);
    q.normalize();
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    rclcpp::Time odom_current_time = this->now();
    odom.header.stamp = odom_current_time;
    odom.pose.pose.position.x = robot_state->x;
    odom.pose.pose.position.y = robot_state->y;
    odom.pose.pose.orientation = odom_quat;

    
    if (use_ekf) {
        // Update covariance in odometry message
        for(int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
        }
        
        odom.pose.covariance[0] = ekf_odometry->getCovariance(0,0);  // x position variance
        odom.pose.covariance[7] = ekf_odometry->getCovariance(1,1);  // y position variance
        odom.pose.covariance[35] = ekf_odometry->getCovariance(2,2); // yaw variance
    } 

    odom.twist.twist = *enc_vel;

    odom_tf.header.stamp = odom_current_time;
    odom_tf.transform.translation.x = robot_state->x;
    odom_tf.transform.translation.y = robot_state->y;
    odom_tf.transform.rotation = odom_quat;

    odom_pub_->publish(odom);
    odom_tf_broadcaster_->sendTransform(odom_tf);
}

void Core::encVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    *enc_vel = *msgs;

    rclcpp::Time odom_current_time = this->now();
    current_time = odom_current_time.seconds();
    dt = current_time - last_time;

    if (dt <= 0) return;

    enc_vel->linear.x *= enc_vel_gain;
    enc_vel->angular.z *= enc_wvel_gain;
    
    // Update odometry based on selected mode
    if (use_ekf) {
        updateEKFOdometry(dt);
    } else {
        updateBasicOdometry(dt);
    }

    publishOdometry();
    last_time = current_time;
}

// --- State Machine Management ---

// stateMachineCallback: Updates state based on /state_machine messages from Interface.
void Core::stateMachineCallback(const std_msgs::msg::String::SharedPtr msg)
{
    static const std::unordered_map<std::string, State> state_map = {
        {"INIT", State::INIT},
        {"IDLE", State::IDLE},
        {"AUTO", State::AUTO},
        {"MANUAL", State::MANUAL},
        {"DOCKING", State::DOCKING},
        {"UNDOCKING", State::UNDOCKING},
        {"CHARGING", State::CHARGING},
        {"MANIPULATION", State::MANIPULATION},
        {"EMERGENCY", State::EMERGENCY},
        {"STOP", State::STOP},
        {"SLAM", State::SLAM}
    };

    std::string state_str = msg->data;
    auto it = state_map.find(state_str);
    if (it == state_map.end()) {
        RCLCPP_WARN(this->get_logger(), "Received unknown state string: %s", state_str.c_str());
        return;
    }

    State new_state = it->second;

    if (state.first != new_state) {
        RCLCPP_INFO(this->get_logger(), "State changed (Interface active): %d -> %d",
                    static_cast<int>(state.first), static_cast<int>(new_state));
        state.first = new_state;
    }

    if (state_machine.find(state.first) != state_machine.end()) {
        (*state_machine[state.first])();
    }
    interface_active_ = true;
    last_state_update_time_ = this->now();
}


// stateBackupCallback: Fallback if no /state_machine messages are received.
void Core::stateBackupCallback()
{
    if (!interface_active_) {
        RCLCPP_WARN(this->get_logger(), "No state updates from Interface. Using internal state machine.");
        if (state_machine.find(state.first) != state_machine.end()) {
            (*state_machine[state.first])();
        }
    }
    if ((this->now() - last_state_update_time_).seconds() > 5.0) {
        RCLCPP_WARN(this->get_logger(), "Interface inactive for 5s. Switching to internal state control.");
        interface_active_ = false;
    }
}

// Legacy timer callback for state machine management.
void Core::stateTimerCallback()
{
    auto it = state_machine.find(state.first);
    if (it != state_machine.end() && it->second) {
        (*it->second)();
    } else {
        RCLCPP_WARN(this->get_logger(), "No state function registered for state: %d", static_cast<int>(state.first));
    }
    
    if (state.second != state.first)
        state_before = state.second;
    state.second = state.first;
    
    static int cnt = 0;
    if (++cnt > 10) {
        RCLCPP_INFO(this->get_logger(), "[%s] Current state: %d, Last state: %d, Before state: %d",
                    this->get_name(), static_cast<int>(state.first),
                    static_cast<int>(state.second), static_cast<int>(state_before));
        cnt = 0;
    }
}

// --- State Functions ---
void Core::stateInit()
{
    profiler->resetSpeed(*ctr_vel);
    state.first = State::INIT;
}

void Core::stateIdle()
{
    profiler->resetSpeed(*ctr_vel);
    *ctr_vel = geometry_msgs::msg::Twist();
    // state.first = State::IDLE;
}

void Core::stateAuto()
{
    // profiler->resetSpeed(*ctr_vel);
    // *ctr_vel = profiler->calc(*nav_vel);
    *ctr_vel = *nav_vel;

}

void Core::stateManual()
{
    if (mode_manager_->getState() == ModeManager::State::JOYSTICK) {
        *ctr_vel = profiler->calc(*joy_vel);
    } else {
        *ctr_vel = profiler->calc(*manual_vel);
    }

    // *ctr_vel = profiler->calc(*manual_vel);
    
}

void Core::stateDocking()
{
    *ctr_vel = *docking_vel;
}

void Core::stateUndocking()
{
    *ctr_vel = *nav_vel;
}

void Core::stateCharging()
{
    *ctr_vel = geometry_msgs::msg::Twist();
}

void Core::stateManipulating()
{
    *ctr_vel = geometry_msgs::msg::Twist();
}

void Core::stateStop()
{
    profiler->resetSpeed(*ctr_vel);
    *ctr_vel = profiler->calc(geometry_msgs::msg::Twist());
}

void Core::stateEmergency()
{
    *ctr_vel = geometry_msgs::msg::Twist();
}

// CAN MODE TRUE
void Core::canFrameCallback(const can_frame& frame) {

    if ((frame.can_id & 0x1FFFFFFF) == static_cast<uint32_t>(Motor_driver)) {
        int16_t left_rpm = static_cast<int16_t>(frame.data[2]) | (static_cast<int16_t>(frame.data[3]) << 8);
        int16_t right_rpm = static_cast<int16_t>(frame.data[5]) | (static_cast<int16_t>(frame.data[6]) << 8);
        double v, w;
        
        // RCLCPP_INFO(this->get_logger(), "[CAN] Received RPM: left = %d, right = %d", left_rpm, right_rpm);
        Kinematics::calculateVW(left_rpm, right_rpm, wheel_radius, distance_wheels, gear_ratio, v, w);
        
        // RCLCPP_INFO(this->get_logger(), "[CAN] Calculated velocity: v = %f, w = %f", v, w);
        enc_vel->linear.x = v;
        enc_vel->angular.z = w;

        // odometry 계산 및 publishOdometry() 호출 (encVelCallback과 동일하게)
        rclcpp::Time odom_current_time = this->now();
        current_time = odom_current_time.seconds();
        dt = current_time - last_time;
        
        if (dt > 0) {
            enc_vel->linear.x *= enc_vel_gain;
            enc_vel->angular.z *= enc_wvel_gain;
            
            if (use_ekf) {
                updateEKFOdometry(dt);
            } else {
                updateBasicOdometry(dt);
            }

            publishOdometry();
            last_time = current_time;
        }
    }
}

} // namespace AMR

