#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "san_msgs/action/set_state.hpp"

class RobotStateActionClient : public rclcpp::Node
{
public:
    using SetState = san_msgs::action::SetState;
    using GoalHandle = rclcpp_action::ClientGoalHandle<SetState>;

    RobotStateActionClient() : Node("robot_state_action_client")
    {
        client_ = rclcpp_action::create_client<SetState>(this, "set_robot_state");
    }

    void send_goal(uint8_t state, uint8_t status)
    {
        auto goal_msg = SetState::Goal();
        goal_msg.state = state;
        goal_msg.status = status;
        client_->async_send_goal(goal_msg);
    }

private:
    rclcpp_action::Client<SetState>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStateActionClient>();
    node->send_goal(64, 0); // DOCKING, RUNNING
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
