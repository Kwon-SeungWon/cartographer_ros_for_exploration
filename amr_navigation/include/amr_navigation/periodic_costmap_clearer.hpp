#ifndef PERIODIC_COSTMAP_CLEARER_HPP
#define PERIODIC_COSTMAP_CLEARER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

class PeriodicCostmapClearer : public rclcpp::Node {
public:
  PeriodicCostmapClearer();
private:
  void clearCostmap();
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr local_costmap_clear_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr global_costmap_clear_client_;
  rclcpp::TimerBase::SharedPtr costmap_clear_timer_;
};

#endif // PERIODIC_COSTMAP_CLEARER_HPP 