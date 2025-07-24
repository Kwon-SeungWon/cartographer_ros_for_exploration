#include "amr_navigation/periodic_costmap_clearer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PeriodicCostmapClearer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 