#include "ros2_laser_merger/laser_scan_merger.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<laser_scan_merger::scanMerger>());
  rclcpp::shutdown();
  return 0;
}