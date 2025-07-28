#ifndef __NODE_MANAGER_HPP__
#define __NODE_MANAGER_HPP__

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <sys/types.h>  // pid_t
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_msgs/srv/load_map.hpp"

#include "amr_interface/simulation.h"

namespace AMR
{

using namespace std::chrono_literals;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

class NodeManager
{
private:
    pid_t nav_pid_;
    pid_t slam_pid_;
    pid_t auto_slam_pid_;

    pid_t launchProcess(const std::string &command);

    bool stopProcessGroup(pid_t pid);

public:
    NodeManager();
    ~NodeManager();

    void runNavigation(std::string map_file);
    void runSLAM();
    void runAutoSLAM();

    bool stopNavigation();

    bool pauseNavigation();
    bool resumeNavigation();

    bool stopSLAM();
    bool stopAutoSLAM();

    void saveMap(std::string map_name);

    bool updateMapServer(const std::string & new_map_file);
};

} // namespace AMR

#endif
