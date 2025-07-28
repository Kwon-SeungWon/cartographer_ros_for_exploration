#ifndef _PARAMETER_UPDATER_HPP_
#define _PARAMETER_UPDATER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <unordered_map>

namespace AMR {

class ParameterUpdater : public rclcpp::Node
{
public:
  ParameterUpdater();

  /// @brief Update target node parameters directly via service call using cached YAML.
  /// @param mode The mode key in the YAML file (e.g. "NORMAL", "DOCK", etc.)
  /// @return true if all updates succeeded.
  bool updateParametersDirectly(const std::string &mode);

  /// @brief Print update result.
  void printUpdateResult(const std::string &mode, bool success);

private:
  // YAML 파일 캐싱 관련 멤버
  YAML::Node cached_yaml_;
  bool yaml_loaded_;
  std::string yaml_file_;

  // 서비스 클라이언트 캐시: target 노드의 set_parameters 서비스 호출용
  std::unordered_map<std::string, rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr> service_clients_;

  // YAML 파일 로드 및 파라미터 업데이트 함수
  bool loadYamlFile(const std::string &file);
  bool loadParams(const std::string &mode);

  // target 노드에 업데이트 요청을 보내는 함수
  void updateParams(const std::vector<rcl_interfaces::msg::Parameter> &params, const std::string &service);

  // 각 파라미터 그룹 업데이트 함수 (업데이트할 파라미터들을 벡터에 모아서 updateParams() 호출)
  bool updateParameterGroup(const YAML::Node &node, const std::string &service, const std::string &group);
  bool updateVectorParameter(const YAML::Node &node, const std::string &service, const std::string &key);
  bool updateBooleanParameters(const YAML::Node &node, const std::string &service);

  // 헬퍼: 파라미터 객체 생성
  rcl_interfaces::msg::Parameter createParameter(const std::string &name, double value);
  rcl_interfaces::msg::Parameter createParameter(const std::string &name, bool value);
  rcl_interfaces::msg::Parameter createParameter(const std::string &name, const std::string &value);

  // 서비스 클라이언트 반환 (캐싱)
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr getServiceClient(const std::string &service);
};

} // namespace AMR

#endif // _PARAMETER_UPDATER_HPP_
