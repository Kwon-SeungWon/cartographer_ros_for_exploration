#include "amr_core/amr_core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto core_node = std::make_shared<AMR::Core>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(core_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
