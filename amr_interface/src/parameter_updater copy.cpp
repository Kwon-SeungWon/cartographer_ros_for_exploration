#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>

#include "san_msgs/srv/update_params.hpp"

class ParameterUpdater : public rclcpp::Node
{
public:
  ParameterUpdater() : Node("parameter_updater"), yaml_loaded_(false)
  {
    this->declare_parameter<std::string>("yaml_file", "");

    if (!this->get_parameter("yaml_file", yaml_file_) || yaml_file_.empty()) {
      std::string package_path = ament_index_cpp::get_package_share_directory("amr_interface");
      yaml_file_ = package_path + "/param/update_param.yaml";
    }

    // YAML 파일을 한 번만 로드해서 캐싱합니다.
    if (loadYamlFile(yaml_file_)) {
      RCLCPP_INFO(this->get_logger(), "Loaded YAML file successfully: %s", yaml_file_.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", yaml_file_.c_str());
    }

    update_service_ = this->create_service<san_msgs::srv::UpdateParams>(
      "update_params",
      std::bind(&ParameterUpdater::serviceCallback, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "ParameterUpdater node is ready.");
    RCLCPP_INFO(this->get_logger(), "Using YAML file: %s", yaml_file_.c_str());
  }

private:
  // 캐싱된 YAML 파일
  YAML::Node cached_yaml_;
  bool yaml_loaded_;
  std::string yaml_file_;

  // 서비스 클라이언트를 서비스 이름별로 캐싱
  std::unordered_map<std::string, rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr> service_clients_;

  rclcpp::Service<san_msgs::srv::UpdateParams>::SharedPtr update_service_;

  // YAML 파일을 캐싱하는 함수
  bool loadYamlFile(const std::string &file)
  {
    try {
      auto start_time = std::chrono::steady_clock::now();
      cached_yaml_ = YAML::LoadFile(file);
      auto end_time = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
      RCLCPP_INFO(this->get_logger(), "YAML file loaded in %.2f ms", elapsed);
      yaml_loaded_ = true;
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML file load error: %s", e.what());
      yaml_loaded_ = false;
      return false;
    }
  }

  // 서비스 클라이언트 캐시에서 가져오거나 새로 생성
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr getServiceClient(const std::string &service)
  {
    if (service_clients_.find(service) == service_clients_.end()) {
      auto client = this->create_client<rcl_interfaces::srv::SetParameters>(service);
      service_clients_[service] = client;
      return client;
    }
    return service_clients_[service];
  }

  void serviceCallback(
    const std::shared_ptr<san_msgs::srv::UpdateParams::Request> request,
    std::shared_ptr<san_msgs::srv::UpdateParams::Response> response)
  {
    auto start_time = std::chrono::steady_clock::now();

    std::string mode = request->mode;
    RCLCPP_INFO(this->get_logger(), "Service call received for mode: %s", mode.c_str());

    if (!yaml_loaded_) {
      if (!loadYamlFile(yaml_file_)) {
        response->success = false;
        response->message = "Failed to load parameters from YAML";
        return;
      }
    }

    if (!loadParams(mode)) {
      response->success = false;
      response->message = "Failed to update parameters from YAML";
      return;
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "Parameters updated in %.2f ms", elapsed_time);

    response->success = true;
    response->message = "Parameters updated from YAML file for mode: " + mode;
  }

  bool loadParams(const std::string &mode)
  {
    if (!cached_yaml_[mode]) {
      RCLCPP_ERROR(this->get_logger(), "Mode %s not found in YAML file.", mode.c_str());
      return false;
    }

    YAML::Node section = cached_yaml_[mode];

    if (section["controller_server"]) {
      YAML::Node ctrlNode = section["controller_server"]["ros__parameters"];
      updateParameterGroup(ctrlNode, "/controller_server/set_parameters", "goal_checker");
      updateParameterGroup(ctrlNode, "/controller_server/set_parameters", "FollowPath");
    }

    if (section["velocity_smoother"]) {
      YAML::Node velNode = section["velocity_smoother"]["ros__parameters"];
      updateVectorParameter(velNode, "max_velocity", "/velocity_smoother/set_parameters");
      updateVectorParameter(velNode, "min_velocity", "/velocity_smoother/set_parameters");
      updateVectorParameter(velNode, "max_accel", "/velocity_smoother/set_parameters");
      updateVectorParameter(velNode, "max_decel", "/velocity_smoother/set_parameters");
    }

    if (section["local_costmap"]) {
      YAML::Node localNode = section["local_costmap"]["local_costmap"]["ros__parameters"];
      updateBooleanParameters(localNode, "/local_costmap/local_costmap/set_parameters");
    }

    if (section["global_costmap"]) {
      YAML::Node globalNode = section["global_costmap"]["global_costmap"]["ros__parameters"];
      updateBooleanParameters(globalNode, "/global_costmap/global_costmap/set_parameters");
    }

    return true;
  }

  void updateParameterGroup(const YAML::Node &node, const std::string &service, const std::string &group)
  {
    if (node[group]) {
      YAML::Node paramsNode = node[group];
      std::vector<rcl_interfaces::msg::Parameter> params;
      for (auto it = paramsNode.begin(); it != paramsNode.end(); ++it) {
        // 여기서는 double 값으로 가정합니다.
        params.push_back(createParameter(group + "." + it->first.as<std::string>(), it->second.as<double>()));
      }
      updateParams(params, service);
    }
  }

  void updateVectorParameter(const YAML::Node &node, const std::string &key, const std::string &service)
  {
    if (node[key]) {
      YAML::Node arrayNode = node[key];
      if (arrayNode.IsSequence() && arrayNode.size() == 3) {
        std::vector<rcl_interfaces::msg::Parameter> params = {
          createParameter(key + ".x", arrayNode[0].as<double>()),
          createParameter(key + ".y", arrayNode[1].as<double>()),
          createParameter(key + ".z", arrayNode[2].as<double>())
        };
        updateParams(params, service);
      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid sequence for %s", key.c_str());
      }
    }
  }

  void updateBooleanParameters(const YAML::Node &node, const std::string &service)
  {
    if (!node) return;
    std::vector<rcl_interfaces::msg::Parameter> params;
    for (auto it = node.begin(); it != node.end(); ++it) {
      if (it->second["enabled"]) {
        params.push_back(createParameter(it->first.as<std::string>() + ".enabled", it->second["enabled"].as<bool>()));
      }
    }
    updateParams(params, service);
  }

  rcl_interfaces::msg::Parameter createParameter(const std::string &name, double value)
  {
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = value;
    return param;
  }

  rcl_interfaces::msg::Parameter createParameter(const std::string &name, bool value)
  {
    rcl_interfaces::msg::Parameter param;
    param.name = name;
    param.value.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param.value.bool_value = value;
    return param;
  }

  void updateParams(const std::vector<rcl_interfaces::msg::Parameter> &params, const std::string &service)
  {
    auto client = getServiceClient(service);

    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_ERROR(this->get_logger(), "Service %s unavailable.", service.c_str());
      return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters_Request>();
    request->parameters = params;
    auto future = client->async_send_request(request);

    if (future.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Updated parameters at %s", service.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to update parameters at %s within timeout.", service.c_str());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterUpdater>());
  rclcpp::shutdown();
  return 0;
}
