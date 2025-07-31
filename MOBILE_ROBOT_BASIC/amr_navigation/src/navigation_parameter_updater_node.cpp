#include "amr_navigation/navigation_parameter_updater.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationParameterUpdater>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
