#include "amr_navigation/periodic_costmap_clearer.hpp"

PeriodicCostmapClearer::PeriodicCostmapClearer()
: Node("periodic_costmap_clearer")
{
  local_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
  global_costmap_clear_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
  costmap_clear_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&PeriodicCostmapClearer::clearCostmap, this));
  RCLCPP_INFO(this->get_logger(), "PeriodicCostmapClearer node started. Will clear local and global costmaps every 3 seconds.");
}

void PeriodicCostmapClearer::clearCostmap()
{
  bool local_ready = local_costmap_clear_client_->wait_for_service(std::chrono::milliseconds(100));
  bool global_ready = global_costmap_clear_client_->wait_for_service(std::chrono::milliseconds(100));
  if (!local_ready) {
    RCLCPP_WARN(this->get_logger(), "Local costmap clear service not available");
  } else {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    local_costmap_clear_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Requested local costmap clear");
  }
  if (!global_ready) {
    RCLCPP_WARN(this->get_logger(), "Global costmap clear service not available");
  } else {
    auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
    global_costmap_clear_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Requested global costmap clear");
  }
} 