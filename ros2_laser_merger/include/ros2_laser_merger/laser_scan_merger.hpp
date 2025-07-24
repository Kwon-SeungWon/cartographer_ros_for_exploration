#ifndef LASER_SCAN_MERGER_HPP
#define LASER_SCAN_MERGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/time.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace message_filters;
using SyncPolicy = sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>;

namespace laser_scan_merger
{
    class scanMerger : public rclcpp::Node
    {
    public:
        scanMerger();
        ~scanMerger();

    private:
        // Parameter Initialization
        void initialize_params();

        // Retrieve Parameters
        void get_params();

        // LaserScan message subscribers
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser1_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> laser2_sub_;

        // Synchronization
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

        // Publisher
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

        // Executor for handling callbacks
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
        std::thread executor_thread_;

        // LaserScan Subscriptions (Optional)
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;

        // Latest LaserScan messages
        sensor_msgs::msg::LaserScan laser1_;
        sensor_msgs::msg::LaserScan laser2_;

        // Callback for synchronized laser scan data
        void synchronized_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser1_msg,
                                   const sensor_msgs::msg::LaserScan::ConstSharedPtr &laser2_msg);

        // Point Cloud Processing
        void update_point_cloud_rgb();

        // Remove points within a certain radius
        void removePointsWithinRadius(pcl::PointCloud<pcl::PointXYZ> &cloud, float radius, float center_x, float center_y);

        // Remove points within a robot basefootprint
        void removePointsWithinRobot(pcl::PointCloud<pcl::PointXYZ>& cloud, float center_x, float center_y);

        // Utility functions
        float GET_R(float x, float y);
        float GET_THETA(float x, float y);
        float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle);

        // Topics and Parameters
        std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
        bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;

        // Laser 1 parameters
        float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
        uint8_t laser1R_, laser1G_, laser1B_;

        // Laser 2 parameters
        float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
        uint8_t laser2R_, laser2G_, laser2B_;
    };
}

#endif // LASER_SCAN_MERGER_HPP
