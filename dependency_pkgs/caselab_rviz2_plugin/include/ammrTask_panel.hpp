#ifndef CASELAB_RVIZ_PLUGIN__TASK_PANEL_HPP_
#define CASELAB_RVIZ_PLUGIN__TASK_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QComboBox> 
#include <QFont> 
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace caselab_rviz_plugin
{

class TaskPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TaskPanel(QWidget * parent = 0);
  ~TaskPanel() override;

  // setTask 에서 사용될 mission, start, goal 입력값은 별도 위젯으로 입력받음.
  // (YAML로 읽어오는 기존 코드는 필요에 따라 유지하거나 제거할 수 있음.)
  // 아래는 buildMap과 setTask 각각의 메시지 전송에 필요한 입력값을 위한 위젯들입니다.
  
  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  // /vda5050/instantactions 토픽에 publish
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // 공통 라벨
  QLabel * label_;

  // --- setTask 입력 위젯 ---
  QLabel * label_setTask;
  QComboBox * missionComboBox_;   // mission: home, move, load, unload 4개 옵션
  QLineEdit * startLineEdit_;
  QLineEdit * goalLineEdit_;
  QPushButton * button_setTask_;

  // --- buildMap 입력 위젯 ---
  QLabel * label_buildMap;
  QLineEdit * le_mapCommand_;
  QLineEdit * le_mapName_;
  QLineEdit * le_mapId_;
  QPushButton * button_buildMap_;

private Q_SLOTS:
  void sendSetTask();    // setTask 메시지 전송 슬롯
  void sendBuildMap();   // buildMap 메시지 전송 슬롯

private:
  // JSON 내부 문자열에 대해 이스케이프 처리하는 헬퍼 함수
  QString escapeJson(const QString & str);
};

}  // namespace caselab_rviz_plugin

#endif  // CASELAB_RVIZ_PLUGIN__TASK_PANEL_HPP_
