#ifndef CASELAB_RVIZ_PLUGIN__CAM_PANEL_HPP_
#define CASELAB_RVIZ_PLUGIN__CAM_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/int32.hpp>

namespace caselab_rviz_plugin
{
class CamPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit CamPanel(QWidget * parent = 0);
  ~CamPanel() override;

  int cnt = 0;
  
  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;


  QLabel * label_;
  QPushButton * button_;

private Q_SLOTS:
  void buttonActivated();
};

}  // namespace caselab_rviz_plugin

#endif  // CASELAB_RVIZ_PLUGIN__CAM_PANEL_HPP_