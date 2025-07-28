#pragma once
#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>


class PathStatusManager {
public:
    PathStatusManager(rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub, rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub2, double threshold, double missed_dist);

    void setWaypoints(const geometry_msgs::msg::PoseArray& waypoints);
    void setNodePath(const std::vector<uint16_t>& node_path);
    void updateStatus(const geometry_msgs::msg::Pose& robot_pose);
    void path_publish();
    void path_status_publish();

private:
    std::vector<uint8_t> path_status_;
    geometry_msgs::msg::PoseArray current_path_;
    std::vector<uint16_t> current_node_path_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr path_status_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr path_pub_;
    double threshold_;
    double missed_dist_;
    std::vector<double> prev_dist_to_next_;
    std::vector<bool> tracking_started_; // Track if we've started monitoring each waypoint
}; 