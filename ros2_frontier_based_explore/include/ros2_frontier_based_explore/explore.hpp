#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_frontier_based_explore/frontier_search.hpp"
#include "ros2_frontier_based_explore/costmap.hpp"
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/empty.hpp"
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include "tf2_ros/transform_listener.h"

#include "visualization_msgs/msg/marker_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"


#include <mutex>

namespace ros2_frontier_based_explore
{

    class Explore : public rclcpp::Node
    {
    public:
        using ClientT = nav2_msgs::action::NavigateToPose;
        using ActionClient = rclcpp_action::Client<ClientT>;

        Explore();

    private:
        void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

        void mapUpdateReceived(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);

        void goalResponseCallback(
            // std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future);
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);
        void reachgoalResponseCallback(
            rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

        void makePlan();

        void goToOrigin();

        void visualizeFrontiers(const std::vector<Frontier> &frontiers);

        bool goalOnBlacklist(const geometry_msgs::msg::Point &goal);
        bool checkFrontier(geometry_msgs::msg::Point goal,std::vector<ros2_frontier_based_explore::Frontier> frontiers);


        geometry_msgs::msg::Pose getRobotPose() const;

        // publishers and subscribers
        rclcpp::TimerBase::SharedPtr exploring_timer_;
        rclcpp::TimerBase::SharedPtr calculating_timer_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr costmap_sub_;
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::ConstSharedPtr costmap_updates_sub_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;

        // action client for nav2
        ActionClient::SharedPtr nav_to_pose_client_;
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;

        void distanceCalculator();
        void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void stop();

        // tf
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // custom classes
        Costmap costmap_;
        FrontierSearch search_;

        // params
        double potential_scale_, orientation_scale_, gain_scale_, min_frontier_size_;
        double planner_frequency_, distance_threshold_, total_distance_;
        bool visualize_;
        int timeout_;
        int check_finish_;
        bool state;
        bool exploration_active_;
        bool return_to_origin_;
        // places you should stop trying to explore
        std::vector<geometry_msgs::msg::Point> frontier_blacklist_;

        // remember prev goal
        geometry_msgs::msg::Point prev_goal_;
        double prev_distance_;
        rclcpp::Time last_progress_;
        int timeout_sec;
        rclcpp::Duration progress_timeout_{0, 0};

        std::mutex mutex_;  // protects sending of actions to nav2
        geometry_msgs::msg::Pose last_pose_;

        std::vector<ros2_frontier_based_explore::Frontier> frontiers;
        geometry_msgs::msg::Point target_position;


    };

} // namespace ros2_frontier_based_explore