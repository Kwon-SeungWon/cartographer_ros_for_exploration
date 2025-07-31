#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "san_msgs/action/set_state.hpp"
#include "san_msgs/msg/state_machine.hpp"

class RobotStateActionServer : public rclcpp::Node
{
public:
    using SetState = san_msgs::action::SetState;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SetState>;

    RobotStateActionServer() : Node("robot_state_action_server"), state_(san_msgs::msg::State::IDLE), status_(san_msgs::msg::State::RUNNING)
    {
        action_server_ = rclcpp_action::create_server<SetState>(
            this, "set_robot_state",
            std::bind(&RobotStateActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotStateActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotStateActionServer::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const SetState::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received request: state=%d, status=%d", goal->state, goal->status);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        state_ = goal->state;
        status_ = goal->status;
        auto result = std::make_shared<SetState::Result>();
        result->success = true;
        result->message = "State updated successfully!";
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<SetState>::SharedPtr action_server_;
    uint8_t state_;
    uint8_t status_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotStateActionServer>());
    rclcpp::shutdown();
    return 0;
}
