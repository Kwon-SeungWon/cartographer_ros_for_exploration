#include "wheelchair_core/wheelchair_core.h"

Wheelchair::Core::Core() : pnh("~"), motor(new Motor{0}), error(new Error{0})
{
    getParam();
    setPublisher();
    setSubscriber();
    Init();
}

Wheelchair::Core::~Core()
{
    if (driver) {
        driver->shutdown();
        driver.reset();
    }
    state_timer_.reset();
    ctr_vel.reset();
    manual_vel.reset();
}

void Wheelchair::Core::getParam()
{
    pnh.param("wheel_radius", wheel_radius,0.3);
    pnh.param("distance_wheels", distance_wheels,0.63);
    pnh.param("gear_ratio", gear_ratio,28.0);
    pnh.param("can_device", can_device,std::string("can0"));
    pnh.param("endcoder_resolution", endcoder_resolution,4096);
    pnh.param("pole_number", pole_number,28);
    pnh.param("loop_hz", loop_hz,20.0);
    pnh.param("can_hz", can_hz,10.0);
    pnh.param("max_rpm", max_rpm,2000);
    pnh.param("hall_offset", hall_offset,240);
    pnh.param("DRIVER_ID", DRIVER_ID,1);
    pnh.param("Main1", Main1,0x1A1B101 );
    pnh.param("Conveyor", Conveyor,0x1A1B103);
    pnh.param("Sub_Conveyor", Sub_Conveyor,0x1B1A103);
    pnh.param("Sub", Sub,0x1B1A101);
    pnh.param("Motordriver", Motordriver,0xB8B701);
    pnh.param("enc_vel_gain", enc_vel_gain,1.0);
    pnh.param("enc_wvel_gain", enc_wvel_gain,0.9);
}

void Wheelchair::Core::setPublisher()
{
    enc_vel_pub_ = nh.advertise<geometry_msgs::Twist>("enc_vel", 10);
    state_manual_pub_ = nh.advertise<std_msgs::Bool>("manual_state", 1);
    state_joystick_pub_ = nh.advertise<std_msgs::Bool>("joystick_state", 1);
    state_auto_pub_ = nh.advertise<std_msgs::Bool>("auto_state", 1);
    state_emergency_pub_ = nh.advertise<std_msgs::Bool>("emergency_state", 1);
}

void Wheelchair::Core::setSubscriber()
{
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("nav_vel", 1, &Wheelchair::Core::cmdVelCallback, this);
    manual_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("manual_vel", 1, &Wheelchair::Core::manualVelCallback, this);
    joy_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("joy_vel", 1, &Wheelchair::Core::joyVelCallback, this);
    ems_sub_ = nh.subscribe<std_msgs::Bool>("/wheelchair/ems_input", 1, &Wheelchair::Core::emergencyCallback, this);
    autorun_sub_ = nh.subscribe<std_msgs::Bool>("/wheelchair/autorun", 1, &Wheelchair::Core::autorunCallback, this);
}

void Wheelchair::Core::Init()
{
    driver = std::make_shared<can::ThreadedSocketCANInterface>();
    if (!driver->init(can_device, 0))
    {
        ROS_FATAL("[Wheelchair_core] Failed to initialize can_device at %s", can_device.c_str());
        ros::shutdown();
        return;
    }
    else
    {
        ROS_INFO("[Wheelchair_core] Successfully connected to %s.", can_device.c_str());
        driver->recover();
    }

    if (driver) {
        frame_listener = driver->createMsgListenerM(this, &Wheelchair::Core::canCallback);
    } else {
        ROS_ERROR("[Wheelchair_core] Failed to initialize CAN driver. Frame listener will not be created.");
    }

    control_timer = std::make_shared<ros::Timer>(
        nh.createTimer(ros::Duration(1.0 / loop_hz), std::bind(&Core::controlMotor, this, std::placeholders::_1))
    );
    state_timer_ = std::make_shared<ros::Timer>(
        nh.createTimer(ros::Duration(1.0 / loop_hz), std::bind(&Core::stateTimerCallback, this, std::placeholders::_1))
    );
    ctr_vel = std::make_shared<geometry_msgs::Twist>();
    manual_vel = std::make_shared<geometry_msgs::Twist>();
    joy_vel = std::make_shared<geometry_msgs::Twist>();

    profiler = std::make_shared<VelocityProfile>(0.2, 0.0, 0.3, loop_hz);

    client = nh.serviceClient<wheelchair_serial::GetMode>("get_select_switch_status");

    state_machine.insert(std::make_pair(State::AUTO, std::make_shared<std::function<void()>>(std::bind(&Wheelchair::Core::stateAuto, this))));
    state_machine.insert(std::make_pair(State::MANUAL, std::make_shared<std::function<void()>>(std::bind(&Wheelchair::Core::stateManual, this))));
    state_machine.insert(std::make_pair(State::EMERGENCY, std::make_shared<std::function<void()>>(std::bind(&Wheelchair::Core::stateEmergency, this))));
    state_machine.insert(std::make_pair(State::JOYSTICK, std::make_shared<std::function<void()>>(std::bind(&Wheelchair::Core::stateJoystick, this))));


    state = std::make_pair(State::INIT, State::INIT);
    state_before = State::INIT;
    manual_flag = false;
    emergency_flag = false;
}

bool Wheelchair::Core::sendCan(int id, char* data)
{
    if (!driver) {
        ROS_ERROR("[Wheelchair_core] CAN driver is not initialized!");
        return false;
    }

    can::Frame can_msgs;
    int32_t header = id | (int32_t)TMID << 8 | (int32_t)RMID << 16;

    can_msgs.id = header;
    can_msgs.is_rtr = false;
    can_msgs.is_extended = true;
    can_msgs.is_error = false;
    can_msgs.dlc = 8;

    for (int index = 0; index < 8; ++index)
        can_msgs.data[index] = data[index];

    bool res = driver->send(can_msgs);

    if (!res)
    {
        ROS_ERROR("[Wheelchair_core] Failed to send CAN message, attempting recovery");
        driver->recover();
    }

    return res;
}

void Wheelchair::Core::cmdVelCallback(const geometry_msgs::TwistConstPtr& msgs)
{
    cmd_vel = *msgs;
    // manual_flag = false;

    // if (state.first != State::AUTO) {
    //     state.first = State::AUTO;
    // }
}

void Wheelchair::Core::manualVelCallback(const geometry_msgs::TwistConstPtr& msgs)
{
    *manual_vel = *msgs;
    manual_flag = true;
    last_manual_time = ros::Time::now();

    if (state.first != State::MANUAL) {
        state.first = State::MANUAL;
    }
}

void Wheelchair::Core::joyVelCallback(const geometry_msgs::TwistConstPtr& msgs)
{
    *joy_vel = *msgs;
    // manual_flag = false;

    // if (state.first != State::JOYSTICK) {
    //     state.first = State::JOYSTICK;
    // }
}

void Wheelchair::Core::emergencyCallback(const std_msgs::BoolConstPtr &msg){
    emergency_flag = true;
    state.first = State::EMERGENCY;
}

void Wheelchair::Core::autorunCallback(const std_msgs::BoolConstPtr& msgs)
{
    emergency_flag = false;
    if (state.first != State::AUTO) {
        state.first = State::AUTO;
    }
}


void Wheelchair::Core::RPM_Calculator()
{
    const double factor = 60 / (2 * M_PI * wheel_radius);
    motor->CMD_L_RPM = round(factor * (ctr_vel->linear.x - (distance_wheels / 2) * ctr_vel->angular.z) * gear_ratio);
    motor->CMD_R_RPM = round(factor * (ctr_vel->linear.x + (distance_wheels / 2) * ctr_vel->angular.z) * gear_ratio);
}

void Wheelchair::Core::controlMotor(const ros::TimerEvent& event)
{
    RPM_Calculator();

    char packit[8];
    int index = 0;

    packit[index++] = PID_PNT_VEL_CMD;     
    packit[index++] = 1;
    packit[index++] = motor->CMD_L_RPM & 0xff;
    packit[index++] = motor->CMD_L_RPM>>8 & 0xff;
    packit[index++] = 1;
    packit[index++] = motor->CMD_R_RPM & 0xff;
    packit[index++] = motor->CMD_R_RPM>>8 & 0xff;
    packit[index++] = 1;                   

    if (sendCan(DRIVER_ID, packit))
    {
        // ROS_INFO("Sent RPM: left = %d, right = %d", motor->CMD_L_RPM, motor->CMD_R_RPM);
    }
}

void Wheelchair::Core::canCallback(const can::Frame& msgs)
{
    if (msgs.id == Motordriver)
    {
        parseRPM(msgs);
    }
}

void Wheelchair::Core::parseRPM(const can::Frame& msgs)
{
    mutex.lock();
    motor->L_RPM = static_cast<int16_t>(msgs.data[2]) | (static_cast<int16_t>(msgs.data[3]) << 8);
    motor->R_RPM = static_cast<int16_t>(msgs.data[5]) | (static_cast<int16_t>(msgs.data[6]) << 8);
    // ROS_INFO("Received RPM: left = %f, right = %f", motor->L_RPM, motor->R_RPM);
    mutex.unlock();
    VW_Calculator();
}

void Wheelchair::Core::VW_Calculator()
{
    mutex.lock();

    motor->real_motor_rpm_left = static_cast<double>(motor->L_RPM) / gear_ratio;
    motor->real_motor_rpm_right = -static_cast<double>(motor->R_RPM) / gear_ratio;
    // ROS_INFO("Received RPM: left = %f, right = %f", motor->real_motor_rpm_left, motor->real_motor_rpm_right);

    motor->real_v = (motor->real_motor_rpm_left + motor->real_motor_rpm_right) * (M_PI * wheel_radius / 60);
    motor->real_w = (motor->real_motor_rpm_right - motor->real_motor_rpm_left) * (M_PI * wheel_radius / (30 * distance_wheels));

    // ROS_INFO("Received RPM: real_v = %f, real_w = %f", motor->real_v, motor->real_w);
    mutex.unlock();
    enc_vel.linear.x = motor->real_v * enc_vel_gain;
    enc_vel.angular.z = motor->real_w * enc_wvel_gain;
    enc_vel_pub_.publish(enc_vel);
}

void Wheelchair::Core::stateTimerCallback(const ros::TimerEvent& event)
{
    if (manual_flag && (ros::Time::now() - last_manual_time > ros::Duration(3.0))) {
        manual_flag = false;
        ROS_INFO("Manual input timeout: resetting manual_flag to false");
    }

    checkMode();
    
    auto state_function = state_machine[state.first];
    if (state_function)
    {
        (*state_function)();
    }

    if (state.second != state.first)
    {
        state_before = state.second;
    }
    state.second = state.first;

    static int cnt = 0;
    if (cnt++ > 10)
    {
        ROS_INFO("[%s] Current state: %d, Last state: %d, Before state: %d",
                 ros::this_node::getName().c_str(), static_cast<int>(state.first),
                 static_cast<int>(state.second), static_cast<int>(state_before));
        cnt = 0;
    }
}

void Wheelchair::Core::checkMode()
{
    if (emergency_flag){
        state.first = State::EMERGENCY;
        return;
    }

    if (!client.exists()) {
        ROS_ERROR("Service get_select_switch_status does not exist!");
        return;
    }

    wheelchair_serial::GetMode srv;
    if (client.call(srv))
    {
        if (manual_flag) {
            if (state.first != State::MANUAL) {
                state.first = State::MANUAL;
                ROS_INFO("State changed to: MANUAL (manual override active)");
            }
            return;
        }

        if (srv.response.joystick) {
            if (state.first != State::JOYSTICK) {
                state.first = State::JOYSTICK;
                ROS_INFO("State changed to: JOYSTICK");
            }
            return;
        }

        if (srv.response.autonomous) {
            if (state.first != State::AUTO) {
                state.first = State::AUTO;
                ROS_INFO("State changed to: AUTO (Autonomous activated)");
            }
            return;
        }

        if (state.first != State::AUTO) {
            state.first = State::AUTO;
            ROS_INFO("State changed to: AUTO (Default fallback)");
        }
    }
    // if (client.call(srv))
    // {
    //     state.first = srv.response.joystick ? State::JOYSTICK : State::AUTO;
    // }
    else
    {
        ROS_ERROR("CORE: Failed to call service get_select_switch_status");
    }
}

void Wheelchair::Core::stateAuto()
{
    std_msgs::Bool msg;
    ROS_INFO("---------AUTO---------");
    msg.data = true;
    state_auto_pub_.publish(msg);

    if (state.first != state.second)
    {
        profiler->resetSpeed(*ctr_vel);
    }

    *ctr_vel = cmd_vel;
}

void Wheelchair::Core::stateJoystick()
{
    std_msgs::Bool msg;
    // ROS_INFO("---------JOYSTICK---------");
    msg.data = true;
    state_joystick_pub_.publish(msg);

    if (state.first != state.second)
    {
        profiler->resetSpeed(*ctr_vel);
    }
    *ctr_vel = profiler->calc(*joy_vel);
}

void Wheelchair::Core::stateManual()
{
    std_msgs::Bool msg;
    ROS_INFO("---------MANUAL---------");

    msg.data = true;
    state_manual_pub_.publish(msg);

    if (state.first != state.second)
    {
        // profiler->resetSpeed(*ctr_vel);
    }
    *ctr_vel = profiler->calc(*manual_vel);
}

void Wheelchair::Core::stateEmergency()
{
    std_msgs::Bool msg;
    ROS_INFO("---------EMERGENCY---------");

    msg.data = true;
    state_emergency_pub_.publish(msg);

    if (state.first != state.second)
    {
        ctr_vel.reset();
        profiler->resetSpeed(*ctr_vel);
        ROS_INFO("Reset the velocity");
    }
    ctr_vel.reset();
    profiler->resetSpeed(*ctr_vel);
}

void Wheelchair::Core::setCurrentState(CurrentState new_state) {
  state.first = new_state;
}

Wheelchair::Core::CurrentState Wheelchair::Core::getCurrentState() const {
  return state.first;
}

void Wheelchair::Core::setLastState(LastState new_state) {
  state.second = new_state;
}

Wheelchair::Core::LastState Wheelchair::Core::getLastState() const {
  return state.second;
}