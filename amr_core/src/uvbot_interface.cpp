#include "uvbot_interface/uvbot_interface.h"

uvbot::Interface::Interface() : pnh("~")
{
    setObject();
    setParam();
    setSubscriber();
    setPublisher();
    setService();

    execution->pubTTS(3); // 준비 완료 되었습니다.
}

uvbot::Interface::~Interface()
{
    delete planner;
    delete execution;
    delete maps;
    delete state;
    delete notice;
}

//Init func
void uvbot::Interface::setObject()
{
    maps = new Map;
    notice = new Notice();
    state = new State(notice);
    planner = new Planner(maps,notice);
    visualization = new Visualization(maps);
    execution = new Execution(maps,notice);
    recharging_timer = nh.createTimer(ros::Duration(1.0), &uvbot::Interface::rechargingTimerCallback,this);
    state_timer = nh.createTimer(ros::Duration(0.02), &uvbot::Interface::stateMachine, this);

}

void uvbot::Interface::setParam()
{
    pnh.getParam("node_folder", node_folder);
    pnh.getParam("map_name", map_name);
    pnh.getParam("map_folder", map_folder);
    pnh.param("battery_limit", battery_limit, 0.15);
    pnh.param("battery_limit_behavior", battery_limit_behavior, 0.3);
    pnh.param("front_home_node", front_home_node,1);
    
    current_map_name = map_name;
    default_map_name = map_name;
    same_map = true;
    first_call_current_map = true;
    inhome = false; // in charging
    around_home = true;
    human_detection_tmp = false;
    uv_human_detect_stop_flag = false;
    clear_count = 0;
    charging_timeout = 0;
    battery_percentage = 1.0;
    charging_state = 0;
    redocking_flag = false;
    charge_tts_flag = false;
    go_charging_low_battery = false;
    empty_plan_check_count = 0;
    redocking_state = false;
    redocking_count = 0;
    docking_tts_flag = false;
    battery_log_timer = 0;
    recharging_count = 0;
    recharging_flag = false;
    docking_error = 99;

}

void uvbot::Interface::setSubscriber()
{
    stop_button_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/button/stop/request",1,&uvbot::Interface::stopButtonCallback,this);
    emergency_button_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/button/emergency/request",1,&uvbot::Interface::emergencyButtonCallback,this);
    manual_button_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/button/manual/request",1,&uvbot::Interface::manualButtonCallback,this);
    home_button_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/button/home/request",1,&uvbot::Interface::homeButtonCallback,this);
    wall_following_button_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/button/wall_following/request",1,&uvbot::Interface::wallFollowingButtonCallback,this);
    behavior_sub_ = nh.subscribe<std_msgs::Int16MultiArray>(ros::this_node::getName()+"/behavior/request",1,&uvbot::Interface::behaviorCallback,this);
    map_list_req_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/map/list/request",1,&uvbot::Interface::mapListReqCallback,this);
    map_select_sub_ = nh.subscribe<std_msgs::String>(ros::this_node::getName()+"/map/select/request",1,&uvbot::Interface::mapSelectCallback,this);
    inhome_sub_ = nh.subscribe<std_msgs::Bool>("uvbot_core/inhome",1,&uvbot::Interface::inhomeCallback,this);
    docking_error_sub_ = nh.subscribe<std_msgs::Int32>("uvbot_charging/docking_error",1,&uvbot::Interface::dockingErrorCallback,this);
    battery_sub_ = nh.subscribe<sensor_msgs::BatteryState>("uvbot_core/battery",1,&uvbot::Interface::batteryCallback,this);
    uvbot_state_sub_ = nh.subscribe<std_msgs::Bool>("uvbot_core/relay_state",1,&uvbot::Interface::relayStateCallback, this);
    uvbot_floor_sub_ = nh.subscribe<uvbot_msgs::pir>("uvbot_core/pir",1,&uvbot::Interface::floorCallback, this);
    map_name_req_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/map/name/request",1,&uvbot::Interface::mapNameCallback, this);
    amcl_pose_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,&uvbot::Interface::amclPoseCallback, this);
    reload_current_map_sub_ = nh.subscribe<std_msgs::String>("uvbot_mapping/reload_current_map",1,&uvbot::Interface::reloadCurrentMapCallback, this);
    default_map_name_req_sub_ = nh.subscribe<std_msgs::Bool>(ros::this_node::getName()+"/default_map_name/request",1,&uvbot::Interface::defaultMapNameCallback, this);
    set_default_map_sub_ = nh.subscribe<std_msgs::String>("/uvbot_mapping/set_default_map", 2, &uvbot::Interface::setDefaultMapCB, this);
}

void uvbot::Interface::setPublisher()
{
    ROS_INFO("set pub");

    stop_button_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/button/stop/response",1);
    emergency_button_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/button/emergency/response",1);
    home_button_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/button/home/response",1);
    manual_button_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/button/manual/response",1);
    wall_following_button_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/button/wall_following/response",1);
    behavior_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/behavior/response",1);
    map_list_pub_ = nh.advertise<std_msgs::String>(ros::this_node::getName()+"/map/list/response",1);
    manual_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/manual_msgs",1);
    initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
    amcl_init_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1);
    wallfollow_stop_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/stop/wallfollow",1);
    docking_stop_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/stop/docking",1);
    watch_pub_ = nh.advertise<std_msgs::String>(ros::this_node::getName()+"/watch",1);
    charge_relay_sw_pub_ = nh.advertise<std_msgs::Bool>("/uvbot_charging/relay",1);
    charger_sw_pub_ = nh.advertise<std_msgs::Bool>("/uvbot_charging/charger",1);
    map_name_res_pub_ = nh.advertise<std_msgs::String>(ros::this_node::getName()+"/map/name/response",1);
    jetson_shutdown_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/jetson_shutdown",10);
    redocking_state_pub_ = nh.advertise<std_msgs::Bool>(ros::this_node::getName()+"/redocking_state",1);
    default_map_name_res_pub_ = nh.advertise<std_msgs::String>(ros::this_node::getName()+"/default_map_name/response",1);

}

void uvbot::Interface::setService()
{
    grid_map_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    load_map_client_ = nh.serviceClient<nav_msgs::LoadMap>("change_map");
    set_map_client_ = nh.serviceClient<nav_msgs::SetMap>("set_map");
    clear_map_client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps"); 
}

// State Machine 
void uvbot::Interface::stateMachine(const ros::TimerEvent& event)
{
    switch(state->getCurrent())
    {
        case INIT:
            stateInit();
            break;

        case DOCKING:
            stateDocking();
            break;
        
        case CHARGING:
            stateCharge();
            break;

        case WORKING:
            stateWorking();
            break;

        case STOP:
            stateStop();
            break;

        case W_STOP:
            stateWorkingStop();
            break;

        case EMERGENCY:
            stateEmergency();
            break;

        case MANUAL:
            stateManual();
            break;

        case WALL_FOLLOWING:
            stateWallFollowing();
            break;
    }
    state->pub();
    // clear the local costmap periodically
    if(clear_count ++ == 500)
    {
        clearMap();
        clear_count = 0;
    }
    std_msgs::Bool tmp;
    tmp.data = redocking_state;
    redocking_state_pub_.publish(tmp);
}

void uvbot::Interface::stateInit()
{
    state->setState(DOCKING);
    execution->dockingStart();
    execution->pubAir(false);
}

void uvbot::Interface::stateDocking()
{
    recharging_flag = false;
    if(!docking_tts_flag)
    {
        docking_tts_flag = true;
        execution->pubTTS(32); // 도킹을 시작합니다.
    }

    if(!execution->checkDocking())
    {
        #if DEBUG_MODE
            ROS_INFO("[uvbot_interface/interface] Docking is success");
        #endif
        // notice->pubLog("log|[uvbot_interface/interface] Docking is success");

        // Docking is finished. Turn on charge relay and charger
        std_msgs::Bool relay_msg;
        relay_msg.data = true;
        charge_relay_sw_pub_.publish(relay_msg);

        docking_tts_flag = false;

        state->setState(CHARGING);
        //execution->pubTTS(31); // 충전을 시작합니다.
    }
    else
    {
        // turn off charge relay and charger
        std_msgs::Bool msg;
        msg.data = false;
        charger_sw_pub_.publish(msg);
        charge_relay_sw_pub_.publish(msg);
    }
    first_call_current_map = false;
    redocking_state = false;
}

void uvbot::Interface::stateCharge()
{
    execution->pubAir(false);

    std_msgs::Bool msg;
    msg.data = relay_state;
    
    // when relay is off
    if(!msg.data && recharging_flag == false)
    {
        // turn on charge relay
        std_msgs::Bool relay_msg;
        relay_msg.data = true;//inhome;
        charge_relay_sw_pub_.publish(relay_msg);
    }
    charger_sw_pub_.publish(msg);

    if(charging_state == 1) // || charging_state == 4)
    {
        redocking_flag = true;
        if(charge_tts_flag == false)
        {
            execution->pubTTS(31); // 충전을 시작합니다.
            redocking_count = 0;
            charge_tts_flag = true;
            notice->pubLog("log|[uvbot_interface/interface] START_CHARGING");
        }
    }
    if(redocking_count >= 15)
    {
        notice->pubLog("log|[uvbot_interface/interface]Charging station error... shut_down..");
        shutdown_jetson.data = true;
        jetson_shutdown_pub_.publish(shutdown_jetson);
        ros::Duration(5.0).sleep();
        // redocking_flag = false;
    }

    if(redocking_flag == false)
    {
        charging_timeout++;
        if(docking_error == 1) //can not detect charging station
        {
            redocking_state = true;
            charge_tts_flag = false;
            execution->pubTTS(33);//도킹에 실패하였습니다. 다시 도킹합니다.
            if(!plan.empty())
            {
                execution->stop();
            }
            behavior = queue<int>();
            plan = queue<int>();
            behavior.push(front_home_node);
            state->setState(WORKING);
            docking_error = 99;
            redocking_count ++;
            ROS_FATAL("[uvbot_interface/interface] re docking");
            notice->pubLog("log|[uvbot_interface/interface] Re-docking error_1 cnt : " + std::to_string(redocking_count));
        }
        else if(docking_error == 2) //can not contact charging pad
        {
            redocking_state = true;
            charge_tts_flag = false;
            execution->pubTTS(33);//도킹에 실패하였습니다. 다시 도킹합니다.
            if(!plan.empty())
            {
                execution->stop();
            }
            behavior = queue<int>();
            plan = queue<int>();
            behavior.push(front_home_node);
            state->setState(WORKING);
            docking_error = 99;
            redocking_count ++;
            ROS_FATAL("[uvbot_interface/interface] re docking");
            notice->pubLog("log|[uvbot_interface/interface] Re-docking error_2 cnt : " + std::to_string(redocking_count));
        }
        else if(docking_error == 0)
        {
            if(charging_timeout > 1500) // can not start charging //30sec
            {
                redocking_state = true;
                charging_timeout = 0;
                charge_tts_flag = false;
                execution->pubTTS(33);//도킹에 실패하였습니다. 다시 도킹합니다.
                if(!plan.empty())
                {
                    execution->stop();
                }
                behavior = queue<int>();
                plan = queue<int>();
                behavior.push(front_home_node);
                // behavior.push(maps->getHome());
                state->setState(WORKING);
                docking_error = 99;
                redocking_count ++;
                ROS_FATAL("[uvbot_interface/interface] re docking");
                notice->pubLog("log|[uvbot_interface/interface] Re-docking error_3 cnt :" + std::to_string(redocking_count));
            }
        }
        
    }
    else
    {
        charging_timeout = 0;
    }
}

void uvbot::Interface::stateWorking()
{
    execution->pubAir(true);
    if(!behavior.empty() && plan.empty() && !execution->getState())
    {
        planner->makePlan(execution->getCurrent(),&behavior);
        plan = planner->getPlan();
        execution->run(&plan);
    }
    else if(behavior.empty() && plan.empty() && !execution->getState())
    {
        around_home = execution->aroundHome();
        if(around_home)
        {
            empty_plan_check_count = 0;
            state->setState(DOCKING);
            execution->dockingStart();
        }
        else
        {
            empty_plan_check_count++;
            if(empty_plan_check_count == 2)
            {
                notice->pubLog("log|[uvbot_interface/interface] Robot is stuck, fail to return home. Shutdown...");
                notice->pubLog("dis|Robot is stuck. shutdown..," + std::to_string(execution->getCurrent()) + "," + std::to_string(0));
                // turn off UV
                execution->pubUV(false);

                // shutdown jetson
                shutdown_jetson.data = true;
                jetson_shutdown_pub_.publish(shutdown_jetson);
            }
            else if (empty_plan_check_count < 2)
            {
                #if DEBUG_MODE
                    ROS_INFO("[uvbot_interface/interface] Robot is not around home, return home");
                #endif
                notice->pubLog("log|[uvbot_interface/interface] Robot is not around home, return home. count: " + std::to_string(empty_plan_check_count));
                if(!plan.empty())
                {
                    execution->stop();
                }
                behavior = queue<int>();
                plan = queue<int>();
                execution->setNext(execution->getCurrent());
                gohome();
            }
        }
    }
    else if(!plan.empty() && !execution->getState())
        execution->run(&plan);
    
    // clear the local costmap periodically
    // if(clear_count ++ == 200)
    // {
    //     clearMap();
    //     clear_count = 0;
    // }
    bool human_detection = execution->getHumanDetect();
    // if(human_detection && !human_detection_tmp)
    //     execution->pubTTS(12); // 지나갑니다 비켜주세요
    human_detection_tmp = human_detection;

    // turn off charge relay and charger
    std_msgs::Bool msg;
    msg.data = false;
    charger_sw_pub_.publish(msg);
    charge_relay_sw_pub_.publish(msg);
    // if(execution->getHumanDetect() && execution->getCurrentNodeType()== 3)
    // {
    //     state->setState(W_STOP);
    //     uv_human_detect_stop_flag = true;
    // }
    first_call_current_map = false;
}

void uvbot::Interface::stateStop()
{
    execution->pubAir(false);
    execution->stop();
}

void uvbot::Interface::stateWorkingStop()
{
    execution->pubAir(false);
    execution->stop();
    // if(uv_human_detect_stop_flag && !execution->getHumanDetect())
    // {
    //      state->setState(WORKING);
    //     uv_human_detect_stop_flag = false;
    // }
}

void uvbot::Interface::stateEmergency()
{
    execution->pubAir(false);
    std_msgs::Bool manual_msg;
    std_msgs::Bool relay_msg;
    manual_msg.data = false;
    manual_pub_.publish(manual_msg);
    execution->emergency();
    if(inhome)
    {
        // Robot is home. Turn on charge relay and charger
        manual_msg.data = true;
        relay_msg.data = true;
        manual_pub_.publish(manual_msg);
        charge_relay_sw_pub_.publish(relay_msg);
        state->setState(CHARGING);
        execution->pubTTS(31); // 충전을 시작합니다.
    }
}   

void uvbot::Interface::stateManual()
{
    execution->pubAir(false);
    // turn off charge relay and charger
    std_msgs::Bool msg;
    msg.data = false;
    charger_sw_pub_.publish(msg);
    charge_relay_sw_pub_.publish(msg);
}

void uvbot::Interface::stateWallFollowing()
{
    execution->pubAir(true);
    if(!execution->checkWallFollow())
    {
        #if DEBUG_MODE
            ROS_INFO("[uvbot_interface/interface] Wall follow is done");
        #endif
        notice->pubLog("log|[uvbot_interface/interface] Wall follow is done");
        state->setState(MANUAL);
    }
}

// callback func
void uvbot::Interface::stopButtonCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    std_msgs::Bool response;
    
    if(state->getCurrent() == DOCKING || state->getCurrent() == WALL_FOLLOWING || state->getCurrent() == STOP)
    {
        redocking_flag = false;
        charge_tts_flag = false;
        if(state->getCurrent() == STOP)
        {
            state->beforeState();
            if(state->getCurrent() == DOCKING)
                stopRequest(docking_stop_pub_,false);
            else if(state->getCurrent() == WALL_FOLLOWING)
                stopRequest(wallfollow_stop_pub_,false);
            response.data = false;
        }
        else if(state->getCurrent() != STOP)
        {
            state->setState(STOP);
            if(state->getCurrent() == DOCKING)
                stopRequest(docking_stop_pub_,true);
            else if(state->getCurrent() == WALL_FOLLOWING)
                stopRequest(wallfollow_stop_pub_,true);
            response.data = true;
        }        
    }
    else if(state->getCurrent() == WORKING || state->getCurrent() == W_STOP)
    {
        if(state->getCurrent() == W_STOP)
        {
            state->beforeState();
            response.data = false;
        }
        else if(state->getCurrent() != W_STOP)
        {
            state->setState(W_STOP);
            execution->stopAction();
            execution->stop();
            response.data = true;
        }
    }
    else
        response.data = false;

    stop_button_pub_.publish(response);    
}

void uvbot::Interface::mapNameCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    std_msgs::String str;
    str.data = current_map_name.c_str();
    map_name_res_pub_.publish(str);
}

void uvbot::Interface::emergencyButtonCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    std_msgs::Bool response;
    if(state->getCurrent() != EMERGENCY)
    {
        execution->pubUV(false);
        state->setState(EMERGENCY);
        execution->stopAction();
        execution->emergency();
        response.data = true;
    }
    else
        response.data = false;
    emergency_button_pub_.publish(response);
}   

void uvbot::Interface::homeButtonCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    std_msgs::Bool response;
    ROS_INFO("[uvbot_interface/interface] home button callback");
    if(state->getCurrent() == W_STOP || state->getCurrent() == WORKING)
    {
        ROS_INFO("[uvbot_interface/interface] home button true");
        execution->pubTTS(10); // 충전스테이션으로 복귀합니다.
        execution->pubUV(false);
        mutex.lock();
        if(!plan.empty())
        {
            execution->stop();
        }
        
        behavior = queue<int>();
        plan = queue<int>();
        gohome();
        // behavior.push(front_home_node);
        mutex.unlock();
        state->setState(WORKING);
        response.data = true;
    }
    else
    {
        response.data = false;
        ROS_INFO("[uvbot_interface/interface] home button false");
    }
    ROS_INFO("[uvbot_interface/interface] home end");
    home_button_pub_.publish(response);
}

void uvbot::Interface::manualButtonCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    std_msgs::Bool response;
    std_msgs::Bool sw;
    #if DEBUG_MODE
        ROS_INFO("[uvbot_interface/interface] manual callback");
    #endif
    notice->pubLog("log|[uvbot_interface/interface] manual callback");

    if(state->getCurrent() == MANUAL)
    {
        sw.data = true;
        manual_pub_.publish(sw);
        // if(state->getprevious() == WALL_FOLLOWING)
        //     state->beforeState();
        // else
        execution->pubTTS(26); // 수동 모드를 해제합니다.
        if(!plan.empty())
        {
            execution->stop();
        }
        behavior = queue<int>();
        plan = queue<int>();
        gohome();

        state->setState(WORKING);

        response.data = false;
    }
    else if(state->getCurrent() == CHARGING)
    {
        // turn off charge relay and charger
        redocking_flag = false;
        charge_tts_flag = false;
        std_msgs::Bool msg;
        msg.data = false;
        charger_sw_pub_.publish(msg);
        charge_relay_sw_pub_.publish(msg);
        execution->pubTTS(25); // 수동 모드로 진입 합니다.
        execution->pubUV(false);
        state->setState(MANUAL);
        sw.data = false;
        manual_pub_.publish(sw);
        execution->stop();
        response.data = true;
    }
    else if(state->getCurrent() == W_STOP || state->getCurrent() == STOP)
    {
        state->setState(MANUAL);
        execution->pubTTS(25); // 수동 모드로 진입 합니다.
        sw.data = false;
        manual_pub_.publish(sw);
        execution->stop();
        execution->stopAction();
        response.data = true;
    }
    else if(state->getCurrent() == WALL_FOLLOWING)
    {
        state->setState(MANUAL);
        execution->pubTTS(25); // 수동 모드로 집입 합니다.
        sw.data = false;
        manual_pub_.publish(sw);
        execution->stopAction();
        response.data = true;
    }
    else
        response.data = false;
    manual_button_pub_.publish(response);    
}

void uvbot::Interface::wallFollowingButtonCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    #if DEBUG_MODE
        ROS_INFO("[uvbot_interface/interface] wallFollowing callback");
    #endif
    execution->setGoHomeFlag(false);
    notice->pubLog("log|[uvbot_interface/interface] wallFollowing callback");
    std_msgs::Bool response;
    if(state->getCurrent() == MANUAL)
    {
        if(state->getCurrent() != WALL_FOLLOWING)
        {
            std_msgs::Bool sw;
            state->setState(WALL_FOLLOWING);
            sw.data = true;
            manual_pub_.publish(sw);
            execution->wallFollowStart();
        }
        response.data = true;
    }
    else
        response.data = false;
    wall_following_button_pub_.publish(response);    
}

void uvbot::Interface::behaviorCallback(const std_msgs::Int16MultiArray::ConstPtr & msgs)
{
    redocking_state = false;
    std_msgs::Bool response;
    execution->setGoHomeFlag(false);
    if(state->getCurrent() == CHARGING || state->getCurrent() == W_STOP || state->getCurrent() == WORKING)
    {
        redocking_flag = false;
        charge_tts_flag = false;
        if(battery_percentage < battery_limit_behavior)
        {
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Battery is low");
            #endif
            execution->pubTTS(39); // 배터리가 부족하여 방역을 진행할 수 없습니다.
            notice->pubLog("log|[uvbot_interface/interface] Battery is low");
            notice->pubLog("dis|Behavior planning fail - low battery " + std::to_string(int(battery_percentage*100.0)) + "%" + "," + std::to_string(execution->getCurrent()) + "," + std::to_string(0));
            response.data = false;
            behavior_pub_.publish(response);
            return; 
        }
        else
        {
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Receive behavior");
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] Receive behavior");
            execution->pubTTS(8); // 목표 지점을 지정합니다.
            execution->pubUV(false);
            mutex.lock();
            if(!plan.empty())
            {
                execution->stop();
            }
            behavior = queue<int>();
            plan = queue<int>();
            for(int i=0;i<msgs->data.size();i++)
                behavior.push(msgs->data[i]);
            mutex.unlock();
            state->setState(WORKING);
            response.data = true;

            // turn off charge relay and charger
            std_msgs::Bool msg;
            msg.data = false;
            charger_sw_pub_.publish(msg);
            charge_relay_sw_pub_.publish(msg);
        }
    }
    else
        response.data = false;
    behavior_pub_.publish(response);
}

void uvbot::Interface::inhomeCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    this->inhome = msgs->data;
}

void uvbot::Interface::dockingErrorCallback(const std_msgs::Int32::ConstPtr & msgs)
{
    this->docking_error = msgs->data;
}

void uvbot::Interface::floorCallback(const uvbot_msgs::pir::ConstPtr & msgs)
{
    for(int i=0;i<4;i++)
    {
        if(msgs->sensor[0].state)
            floor_detect[0].push_back(true);
        else
            floor_detect[0] = std::vector<bool>();
    }
    if(floor_detect[0].size()==3 || floor_detect[1].size()==3 || floor_detect[2].size()==3 || floor_detect[3].size()==3)
        state->setState(EMERGENCY);
}

void uvbot::Interface::mapListReqCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    // query map list in node the node folder
    files.clear();
    DIR *dir;
    struct dirent *diread;

    if ((dir = opendir(node_folder.c_str())) != nullptr)
    {
        while ((diread = readdir(dir)) != nullptr)
        {
            std::string tmp(diread->d_name);
            files.push_back(tmp);
        }
        closedir (dir);

        std_msgs::String map_list;
        std::string tmp;
        std::string delim = ".";
        int file_num = 0;
        for (auto file : files)
        {   
            if(file.size() > 2)
            {
                size_t pos = 0;
                pos = file.find(delim);
                std::string map_name = file.substr(0, pos);

                file_num++;
                tmp += "/";
                tmp += map_name;
            }
        }

        map_list.data = std::to_string(file_num) + tmp;
        map_list_pub_.publish(map_list);
        #if DEBUG_MODE
            ROS_INFO("[uvbot_interface/interface] Send map lists. Ready to change a map.");
        #endif
        notice->pubLog("log|[uvbot_interface/interface] Send map lists. Ready to change a map.");        
    }
    else
    {
        #if DEBUG_MODE
            ROS_ERROR("[uvbot_interface/interface] Fail to load map list");
        #endif
        notice->pubLog("log|[uvbot_interface/interface] Fail to load map list",ERROR);
    }
}

void uvbot::Interface::mapSelectCallback(const std_msgs::String::ConstPtr & msgs)
{
    if(msgs->data == "current_map")
    {
        if(first_call_current_map)
        {
            // send map/node data
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Send current map info");
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] Send current map info");

            maps->send();

            // wait for map generation on UI
            // ros::Duration(1.0).sleep();

            // amcl initial pose topic
            geometry_msgs::PoseWithCovarianceStamped init_pose;
            init_pose.header.stamp = ros::Time::now();
            init_pose.header.frame_id = "map";
            init_pose.pose.pose = maps->getHomePose();
            init_pose.pose.covariance[0] = 1.0;
            init_pose.pose.covariance[7] = 1.0;
            init_pose.pose.covariance[35] = 1.0;
            initial_pose_pub_.publish(init_pose);

            // publish amcl_pose for initializing the robot's pose on UI
            amcl_init_pose_pub_.publish(init_pose);
            first_call_current_map = false;
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Success to set init pose at first");
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] Success to set init pose at first");
        }
        else
        {
            // send map/node data
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Send current map info");
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] Send current map info");
            maps->send();

            // wait for map generation on UI
            // ros::Duration(1.0).sleep();

            // send current pose
            geometry_msgs::PoseWithCovarianceStamped init_pose = execution->getPresentPose();
            amcl_init_pose_pub_.publish(init_pose);
            #if DEBUG_MODE
                ROS_INFO("[uvbot_interface/interface] Send current pose %f, %f",init_pose.pose.pose.position.x, init_pose.pose.pose.position.y);
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] Send current pose"+std::to_string(init_pose.pose.pose.position.x)+","+std::to_string(init_pose.pose.pose.position.y));
        }
    }
    else
    {
        bool correct_map_name = false;
        for(auto file: files)
        {
            std::string tmp = msgs->data + ".yaml";
            if(file == tmp)
                correct_map_name = true;
        }

        if(correct_map_name)
        {
            // compare selected map with open map
            if(current_map_name == msgs->data)
                same_map = true;
            else
                same_map = false;

            if(same_map)
            {
                #if DEBUG_MODE
                    ROS_INFO("[uvbot_interface/interface] Request changing map with the same map name.");
                #endif
                // notice->pubLog("log|[uvbot_interface/interface] Request changing map with the same map name.");
            }
            else
            {
                // check whether the robot is home
                if(inhome)
                {
                    // when the robot is at home
                    current_map_name = msgs->data;
                    #if DEBUG_MODE
                        ROS_INFO("[uvbot_interface/interface] Robot is home. Start to change the map %s", current_map_name.c_str());
                    #endif
                    // notice->pubLog("log|[uvbot_interface/interface] Robot is home. Start to change the map " + current_map_name);
                    // change map_server
                    std::string full_map_path = map_folder + current_map_name + ".yaml";
                    nav_msgs::LoadMap srv;
                    srv.request.map_url = full_map_path;
                    if(!load_map_client_.call(srv))
                    {
                        #if DEBUG_MODE
                            ROS_ERROR("[uvbot_interface/interface] Fail to load new map");
                        #endif
                        // notice->pubLog("log|[uvbot_interface/interface] Fail to load new map",ERROR);
                    }
                    else
                    {
                        if(srv.response.result == 0)
                        {
                            #if DEBUG_MODE
                                ROS_INFO("[uvbot_interface/interface] Success to load new map");
                            #endif
                            // notice->pubLog("log|[uvbot_interface/interface] Success to load new map");
                            maps->change(current_map_name); // change
                            execution->nodeUpdate();
                            maps->send();    // send

                            // amcl initial pose topic
                            geometry_msgs::PoseWithCovarianceStamped init_pose;
                            init_pose.header.stamp = ros::Time::now();
                            init_pose.header.frame_id = "map";
                            init_pose.pose.pose = maps->getHomePose();
                            init_pose.pose.covariance[0] = 1.0;
                            init_pose.pose.covariance[7] = 1.0;
                            init_pose.pose.covariance[35] = 1.0;
                            initial_pose_pub_.publish(init_pose);
                            #if DEBUG_MODE
                                ROS_INFO("[uvbot_interface/interface] Success to set new map and init pose");
                            #endif
                            // notice->pubLog("log|[uvbot_interface/interface] Success to set new map and init pose");
                        }
                        else
                        {
                            #if DEBUG_MODE
                                ROS_ERROR("[uvbot_interface/interface] Fail to load new map error code: %d", srv.response.result);
                            #endif
                            // notice->pubLog("log|[uvbot_interface/interface] Fail to load new map error code: "+std::to_string(srv.response.result));


                        }
                    }
                }
                else
                {
                    // Notice to make robot go home
                    #if DEBUG_MODE
                        ROS_INFO("[uvbot_interface/interface] Robot is not home. Move robot to home");
                    #endif
                    // notice->pubLog("log|[uvbot_interface/interface] Robot is not home. Move robot to home");
                    // notice->pubNotice("log|[uvbot_interface/interface] Robot is not home. Move robot to home");
                }        
            }
        }
        else
        {
            #if DEBUG_MODE
                ROS_ERROR("[uvbot_interface/interface] request incorrect map name");
            #endif
            // notice->pubLog("log|[uvbot_interface/interface] request incorrect map name",ERROR);
        }
    }
}

void uvbot::Interface::batteryCallback(const sensor_msgs::BatteryState::ConstPtr & msgs)
{
    battery_percentage = msgs->percentage;
    charging_state = msgs->power_supply_status;

    if(battery_percentage < battery_limit && state->getCurrent() == WORKING && !go_charging_low_battery)
    {
        ROS_INFO("[uvbot_interface/interface] battay is Low");
        execution->pubTTS(10); // 충전스테이션으로 복귀합니다.
        mutex.lock();
        if(!plan.empty())
        {
            execution->stop();
        }
        behavior = queue<int>();
        plan = queue<int>();
        gohome();
        mutex.unlock();
        state->setState(WORKING);
        execution->pubUV(false);
        go_charging_low_battery = true;
        notice->pubLog("dis|homing - low battery " + std::to_string(int(battery_percentage*100.0)) + "%" + "," + std::to_string(execution->getCurrent()) + "," + std::to_string(0));
    }
    else if (battery_percentage >= battery_limit)
        go_charging_low_battery = false;

    // battery percentage log
    if(battery_log_timer ++ >= 1200)
    {
        notice->pubLog("log|[uvbot_interface/interface] Battery : " + std::to_string(int(battery_percentage*100.0)) + "%");
        battery_log_timer = 0;
    }

        
}

void uvbot::Interface::relayStateCallback(const std_msgs::Bool::ConstPtr & msgs)
{
    relay_state = msgs->data;
}

void uvbot::Interface::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgs)
{
    amcl_data.pose = msgs->pose;
}

void uvbot::Interface::clearMap()
{
    #if DEBUG_MODE
        // ROS_INFO("[uvbot_interface/interface] Clear costmap");
    #endif
    // notice->pubLog("log|[uvbot_interface/interface] Clear costmap");
    std_srvs::Empty srv;
    clear_map_client_.call(srv);
}

void uvbot::Interface::stopRequest(ros::Publisher publisher,bool sw)
{
    std_msgs::Bool msgs;
    msgs.data = sw;
    publisher.publish(msgs);
}

void uvbot::Interface::wathPub(std::string str)
{
    std_msgs::String msgs;
    msgs.data = str;
    watch_pub_.publish(msgs);
}

void uvbot::Interface::gohome()
{
    ROS_INFO("[uvbot_interface/interface] go home currnet is %d next is %d",execution->getCurrent(),execution->getNext());
    execution->setGoHomeFlag(true);
    int curr_dist = planner->dijkstra(execution->getCurrent(),maps->getHome()).size();
    int next_dist = planner->dijkstra(execution->getNext(),maps->getHome()).size();
    if(curr_dist>next_dist)
        execution->setCurrent(execution->getNext());
    behavior.push(front_home_node);
    // behavior.push(maps->getHome());
}

void uvbot::Interface::reloadCurrentMapCallback(const std_msgs::String::ConstPtr& msgs)
{
    // reload map with new built map
    ROS_INFO("[uvbot_interface] Reload new generated map.");
    maps->change(msgs->data);
    maps->send();
    current_map_name = msgs->data;
    default_map_name = msgs->data;
}

void uvbot::Interface::defaultMapNameCallback(const std_msgs::Bool::ConstPtr& msgs)
{
    std_msgs::String tmp;
    tmp.data = default_map_name;
    default_map_name_res_pub_.publish(tmp);
}

void uvbot::Interface::setDefaultMapCB(const std_msgs::String::ConstPtr& msgs)
{
    default_map_name = msgs->data;
}

void uvbot::Interface::rechargingTimerCallback(const ros::TimerEvent& event)
{
    enum STATE cur_state = state->getCurrent();
    if(cur_state == CHARGING)
    {
        recharging_count++;
    }
    else
    {
        recharging_count = 0;
    }

    if(recharging_count >= 7200)//7200
    {
        recharging_flag = true;
        std_msgs::Bool relay_msg;
        relay_msg.data = false;
        charge_relay_sw_pub_.publish(relay_msg);
        ros::Duration(20.0).sleep();
        recharging_flag = false;
        
        relay_msg.data = true;
        charge_relay_sw_pub_.publish(relay_msg);
        ros::Duration(15.0).sleep();


        if(charging_state == 1 || battery_percentage >= 0.80)
        {
            ROS_INFO("[uvbot_interface/interface] re_charging start");
            notice->pubLog("log|[uvbot_interface/interface] re_charging start");
            recharging_count = 0;
            recharging_flag = false;
        }
        else if(cur_state == CHARGING)
        {
            relay_msg.data = false;
            charge_relay_sw_pub_.publish(relay_msg);
            recharging_count = 0;
            recharging_flag = false;
            charge_tts_flag = false;
            redocking_flag = false;
            redocking_state = true;

            if(!plan.empty())
            {
                execution->stop();
            }
            behavior = queue<int>();
            plan = queue<int>();
            behavior.push(front_home_node);
            state->setState(WORKING);
            notice->pubLog("log|[uvbot_interface/interface] re_charging error");
            
        }
    }
}