#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include "camswitch_panel.hpp"

namespace caselab_rviz_plugin
{
CamPanel::CamPanel(QWidget * parent) : Panel(parent)
{
  // Create a label and a button, displayed vertically (the V in VBox means vertical)
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("CAMERA CHANNEL");
  label_->setAlignment(Qt::AlignCenter);
  button_ = new QPushButton("camera num: 0");
  button_->setStyleSheet("background-color:green;");

  layout->addWidget(label_,Qt::AlignCenter);
  layout->addWidget(button_,Qt::AlignCenter);

  // Connect the event of when the button is released to our callback,
  // so pressing the button results in the callback being called.
  QObject::connect(button_, &QPushButton::released, this, &CamPanel::buttonActivated);
}

CamPanel::~CamPanel() = default;  

void CamPanel::onInitialize()
{
  // Access the abstract ROS Node and
  // in the process lock it for exclusive use until the method is done.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  // Get a pointer to the familiar rclcpp::Node for making subscriptions/publishers
  // (as per normal rclcpp code)
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::Int32>("/ammr_cam_ch", 10);

}

// When the subscriber gets a message, this callback is triggered,
// and then we copy its data into the widget's label

// When the widget's button is pressed, this callback is triggered,
// and then we publish a new message on our topic.
void CamPanel::buttonActivated()
{
  auto message = std_msgs::msg::Int32();
  cnt = (cnt + 1) % 4;  
  message.data = cnt;

  button_->setText(QString("camera num: %1").arg(message.data));
  button_->setStyleSheet("background-color:green;");
  publisher_->publish(message);
  
}

}  // namespace caselab_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(caselab_rviz_plugin::CamPanel, rviz_common::Panel)
