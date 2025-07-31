#include "redbot02_core/redbot02_core.h"

namespace Redbot {

Core::Core() : Node("redbot_core")
{
    setParam();
    getParam();
    setPublisher();
    setSubscriber();
    Init();
}

Core::~Core()
{
    running_ = false;  
    if (can_receive_thread_.joinable()) {
        can_receive_thread_.join();  
    }  
    close(can_socket_);
}

void Core::Init()
{
    warnig_flag = false;

    ctr_vel.reset();
    cmd_vel.reset();
    enc_vel.reset();
    manual_vel.reset();

    ctr_vel = std::make_shared<geometry_msgs::msg::Twist>();
    cmd_vel = std::make_shared<geometry_msgs::msg::Twist>();
    enc_vel = std::make_shared<geometry_msgs::msg::Twist>();
    manual_vel = std::make_shared<geometry_msgs::msg::Twist>();

    running_ = true;

    std::string can_device;
    this->get_parameter("can_device", can_device);

    // CAN Socket Setting
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error while opening CAN socket");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket opened successfully");

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in getting CAN interface index");
        close(can_socket_);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN interface index retrieved successfully");

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
        close(can_socket_);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "CAN socket bound to interface successfully");

    // CAN Receive Thread Start
    can_receive_thread_ = boost::thread(&Core::receiveCanMessages, this);

    // Init Control timer
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1./loop_hz),  // Set 10hz
        std::bind(&Core::controlTimerCallback, this)  // Callback
    );

    // Velocity porfiler
    profiler = std::make_shared<VelocityProfile>(acc,loop_hz);

    // Init State machine
    state_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1./loop_hz),  // Set 10hz
        std::bind(&Core::stateTimerCallback, this)  // Callback
    );

    state_machine.insert(std::make_pair(State::INIT,std::make_shared<std::function<void()>>(std::bind(&Core::stateInit,this))));
    state_machine.insert(std::make_pair(State::AUTO,std::make_shared<std::function<void()>>(std::bind(&Core::stateAuto,this))));
    state_machine.insert(std::make_pair(State::STOP,std::make_shared<std::function<void()>>(std::bind(&Core::stateStop,this))));
    state_machine.insert(std::make_pair(State::EMERGENCY,std::make_shared<std::function<void()>>(std::bind(&Core::stateEmergency,this))));
    state_machine.insert(std::make_pair(State::MANUAL,std::make_shared<std::function<void()>>(std::bind(&Core::stateManual,this))));
    state = std::make_pair(State::INIT,State::INIT);
    state_before = State::INIT;
}

void Core::setParam()
{
    // In ROS2, declare parameters before getting them
    this->declare_parameter("wheel_radius", 0.114);
    this->declare_parameter("distance_wheels", 0.404);
    this->declare_parameter("gear_ratio", 0.03);
    this->declare_parameter("can_device", "can0");
    this->declare_parameter("id_rpm", 1);
    this->declare_parameter("id_enc", 1);
    this->declare_parameter("mode", 1);
    this->declare_parameter("encoder_resolution", 4096);
    this->declare_parameter("pole_number", 4);
    this->declare_parameter("can_hz", 10.0);
    this->declare_parameter("max_rpm", 200);
    this->declare_parameter("break_option", 1);
    this->declare_parameter("rated_current", 8.0);
    this->declare_parameter("max_current", 16.0);
    this->declare_parameter("hall_offset", 240);
    this->declare_parameter("Main1", 0x1A1B101);
    this->declare_parameter("Conveyor", 0x1A1B103);
    this->declare_parameter("Sub_Conveyor", 0x1B1A103);
    this->declare_parameter("Sub", 0x1B1A101);
    this->declare_parameter("enc_vel_gain", 0.9);
    this->declare_parameter("enc_wvel_gain", 0.93);
    this->declare_parameter("vx_acc", 0.7);
    this->declare_parameter("vy_acc", 0.7);
    this->declare_parameter("w_acc", 0.7);
    this->declare_parameter("loop_hz", 10.0);

    motor = new Motor{0,};
    error = new Error{0,};
    conveyor = new ConveyorInfo{0,};
}

void Core::getParam()
{
    // Fetch parameters from the ROS2 parameter server
    this->get_parameter("wheel_radius", wheel_radius);
    this->get_parameter("distance_wheels", distance_wheels);
    this->get_parameter("gear_ratio", gear_ratio);
    this->get_parameter("can_device", can_device);
    this->get_parameter("id_rpm", id_rpm);
    this->get_parameter("id_enc", id_enc);
    this->get_parameter("mode", mode);
    this->get_parameter("encoder_resolution", encoder_resolution);
    this->get_parameter("pole_number", pole_number);
    this->get_parameter("can_hz", can_hz);
    this->get_parameter("max_rpm", max_rpm);
    this->get_parameter("break_option", break_option);
    this->get_parameter("rated_current", rated_current);
    this->get_parameter("max_current", max_current);
    this->get_parameter("hall_offset", hall_offset);
    this->get_parameter("Main1", Main1);
    this->get_parameter("Conveyor", Conveyor);
    this->get_parameter("Sub_Conveyor", Sub_Conveyor);
    this->get_parameter("Sub", Sub);
    this->get_parameter("enc_vel_gain", enc_vel_gain);
    this->get_parameter("enc_wvel_gain", enc_wvel_gain);
    this->get_parameter("vx_acc", acc.linear.x);
    this->get_parameter("vy_acc", acc.linear.y);
    this->get_parameter("w_acc", acc.angular.z);
    this->get_parameter("loop_hz", loop_hz);

}

void Core::setPublisher()
{
    // can_tx_pub_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);
    // Initialize publisher with ROS2 API
    enc_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("enc_vel", 10);
}

void Core::setSubscriber()
{
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&Core::cmdVelCallback, this, std::placeholders::_1));
    manual_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "manual_vel", 1, std::bind(&Core::manualCallback, this, std::placeholders::_1));
    con_status_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "con_status", 1, std::bind(&Core::conveyorCallback, this, std::placeholders::_1));
}

bool Core::sendCan(int id, char *data)
{
    struct can_frame frame;
    frame.can_id = id | CAN_EFF_FLAG; 
    frame.can_dlc = 8; 
    memcpy(frame.data, data, frame.can_dlc);

    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error in CAN message send to can0");
        return false; 
    }

    // RCLCPP_INFO(this->get_logger(), "Sent CAN message to can0 - ID: 0x%X, DLC: %d", frame.can_id, frame.can_dlc);
    // RCLCPP_INFO(this->get_logger(), "Data: %02X %02X %02X %02X %02X %02X %02X %02X",
    //             frame.data[0], frame.data[1], frame.data[2], frame.data[3],
    //             frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
    return true;
}

void Core::receiveCanMessages()
{
    while (running_) {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN Receive FAIL!!");
            continue;
        }

        auto can_msg = std::make_shared<can_msgs::msg::Frame>();
        can_msg->id = frame.can_id;
        can_msg->is_rtr = (frame.can_id & CAN_RTR_FLAG) ? true : false;
        can_msg->is_extended = (frame.can_id & CAN_EFF_FLAG) ? true : false;
        can_msg->dlc = frame.can_dlc;
        std::copy(frame.data, frame.data + frame.can_dlc, can_msg->data.begin());

        canCallback(can_msg);
    }
}

void Core::canCallback(const can_msgs::msg::Frame::SharedPtr msgs)
{
    uint32_t can_id = msgs->id & 0x1FFFFFFF; // Masking For low bit
    // RCLCPP_INFO(this->get_logger(), "msgs_id: 0x%X", can_id);

    if (can_id == Sub)
        encCallback(msgs);
}


void Core::encCallback(const can_msgs::msg::Frame::SharedPtr msgs)
{
    int16_t enc_vx, enc_w;

    enc_vx = (int16_t)msgs->data[0] | ((int16_t)msgs->data[1] << 8);
    enc_w = (int16_t)msgs->data[4] | ((int16_t)msgs->data[5] << 8);

    enc_vel->linear.x = (double)enc_vx / SIGNIFICANT_FIGURES * enc_vel_gain;
    enc_vel->angular.z = (double)enc_w / SIGNIFICANT_FIGURES * enc_wvel_gain;
    enc_vel_pub_->publish(*enc_vel);

    // RCLCPP_INFO(this->get_logger(), "send enc_vel linear: %f angular: %f", enc_vel->linear.x, enc_vel->angular.z);
}

// Distinguish for Auto and manaul
void Core::controlTimerCallback()
{
    char buf[8];
    int index = 0;

    int16_t cmd_vel_x = ctr_vel->linear.x * SIGNIFICANT_FIGURES;
    int16_t cmd_vel_z = ctr_vel->angular.z * SIGNIFICANT_FIGURES;
    RCLCPP_INFO(this->get_logger(), "send cmd_vel linear: %d angular: %d", cmd_vel_x, cmd_vel_z);

    buf[index++] = (cmd_vel_x & 0xff);
    buf[index++] = (cmd_vel_x >> 8) & 0xff;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index++] = (cmd_vel_z & 0xff);
    buf[index++] = (cmd_vel_z >> 8) & 0xff;
    buf[index++] = 0x00;
    buf[index] = 0x00;

    sendCan(Main1, buf);
}

void Core::cmdVelCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
    // cmd_vel pointer
    *cmd_vel = *msgs;

    // State Change if no stop & emergency -> AUTO
    if(state.first !=  State::STOP && state.first !=  State::EMERGENCY)
    state.first = State::AUTO;
}

void Core::manualCallback(const std::shared_ptr<const geometry_msgs::msg::Twist>& msgs)
{
  if(warnig_flag)
    *manual_vel = profiler->reduceVelocity(*msgs);
  else
    *manual_vel = *msgs;

  // State Change if no stop & emergency -> Manual
  if(state.first !=  State::STOP && state.first !=  State::EMERGENCY)
    state.first = State::MANUAL;
}

// State Machine Manage

void Core::stateTimerCallback()
{
  (*state_machine[state.first])();
  if (state.second != state.first)
  {
    state_before = state.second;
  }
  state.second = state.first;
  
  static int cnt=0;
  if (cnt++>10)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] current state: %d last state: %d before state: %d", this->get_name(), (int)state.first, (int)state.second, (int)state_before);
    cnt=0;
  }
}

void Core::stateInit()
{
    state.first = State::AUTO;
}

void Core::stateAuto()
{
    *ctr_vel = *cmd_vel;
}

void Core::stateStop()
{
    if(state.first != state.second)
        profiler->resetSpeed(*ctr_vel);
    *ctr_vel = profiler->calc(geometry_msgs::msg::Twist());
}

void Core::stateEmergency()
{
    *ctr_vel = geometry_msgs::msg::Twist();
}

void Core::stateManual()
{
    if(state.first != state.second)
        profiler->resetSpeed(*ctr_vel);
    *ctr_vel = profiler->calc(*manual_vel);
}


} // namespace Redbot
