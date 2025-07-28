#include "rclcpp/rclcpp.hpp"
#include "amr_interface/interface.hpp"
#include "amr_interface/execution.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Interface Node (interface.hpp)
    auto Interface_node = std::make_shared<AMR::Interface>();

    // StateExecutor Create -> Interface (Execute the state)
    auto state_executor = std::make_shared<AMR::StateExecutor>();
    Interface_node->setExecutor(state_executor);

    // Execution Node (Docking timeout & Distance Check)
    auto execution_node = std::make_shared<AMR::ExecutionNode>(Interface_node);

    // MultiThread for Interface Node & Execution Node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(Interface_node);
    executor.add_node(execution_node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
