#include "ros2_frontier_based_explore/explore.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ros2_frontier_based_explore::Explore>();

    // MultiThread Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  

    rclcpp::shutdown();
    return 0;
}
