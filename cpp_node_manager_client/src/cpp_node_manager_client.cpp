#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class CppNodeManagerClient : public rclcpp::Node
{
public:
    CppNodeManagerClient() : Node("cpp_node_manager_client")
    {
        // 서비스 클라이언트 생성
        path_planning_client_ = this->create_client<example_interfaces::srv::AddTwoInts>("path_planning");
        load_graph_client_ = this->create_client<std_srvs::srv::Trigger>("load_graph");
        
        RCLCPP_INFO(this->get_logger(), "CppNodeManagerClient initialized");
        
        // 타이머로 주기적으로 경로 계획 요청
        timer_ = this->create_wall_timer(5s, std::bind(&CppNodeManagerClient::timer_callback, this));
    }

    void request_path_planning(int start_node, int target_node)
    {
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = start_node;
        request->b = target_node;

        RCLCPP_INFO(this->get_logger(), "Requesting path planning: %d -> %d", start_node, target_node);

        auto future = path_planning_client_->async_send_request(request);
        
        // 간단한 대기
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
            auto result = future.get();
            if (result->sum > 0)
            {
                RCLCPP_INFO(this->get_logger(), "Path planning successful! Number of waypoints: %ld", result->sum);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Path planning failed or no path found");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call path_planning service");
        }
    }

    void request_load_graph()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        RCLCPP_INFO(this->get_logger(), "Requesting graph load");

        auto future = load_graph_client_->async_send_request(request);
        
        // 간단한 대기
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        {
            auto result = future.get();
            if (result->success)
            {
                RCLCPP_INFO(this->get_logger(), "Graph loaded successfully: %s", result->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to load graph: %s", result->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call load_graph service");
        }
    }

private:
    void timer_callback()
    {
        static int call_count = 0;
        call_count++;
        
        // 첫 번째 호출에서는 그래프 로드
        if (call_count == 1)
        {
            request_load_graph();
        }
        // 그 이후에는 경로 계획 요청 (예시: 노드 1에서 5로)
        else
        {
            request_path_planning(1, 5);
        }
    }

    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr path_planning_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_graph_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CppNodeManagerClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 