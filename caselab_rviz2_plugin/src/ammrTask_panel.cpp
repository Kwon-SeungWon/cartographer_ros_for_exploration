#include "ammrTask_panel.hpp"

#include <QDateTime>
#include <QStringList>
#include <rviz_common/display_context.hpp>
#include <QDebug>

namespace caselab_rviz_plugin
{

TaskPanel::TaskPanel(QWidget * parent)
  : Panel(parent)
{
  // Main vertical layout with some margins and spacing for clarity.
  auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(5, 5, 5, 5);
  layout->setSpacing(10);

  // Top label
  label_ = new QLabel("[AMMR Task Panel]");
  QFont font = label_->font();
  font.setBold(true);
  label_->setFont(font);
  label_->setAlignment(Qt::AlignCenter);
  label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // --- setTask section ---
  label_setTask = new QLabel("Enter setTask Data");
  label_setTask->setAlignment(Qt::AlignCenter);
  label_setTask->setFont(font);
  label_setTask->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Mission input: Only 4 preset options.
  missionComboBox_ = new QComboBox();
  missionComboBox_->setEditable(false);
  missionComboBox_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  missionComboBox_->addItem("home");
  missionComboBox_->addItem("move");
  missionComboBox_->addItem("load");
  missionComboBox_->addItem("unload");

  // Start and Goal inputs using QLineEdit.
  startLineEdit_ = new QLineEdit();
  startLineEdit_->setPlaceholderText("Start node (optional)");
  startLineEdit_->setAlignment(Qt::AlignCenter);
  startLineEdit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  goalLineEdit_ = new QLineEdit();
  goalLineEdit_->setPlaceholderText("Goal node (required, int)");
  goalLineEdit_->setAlignment(Qt::AlignCenter);
  goalLineEdit_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  button_setTask_ = new QPushButton("Publish setTask");
  button_setTask_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  auto setTaskLayout = new QVBoxLayout();
  setTaskLayout->setSpacing(5);
  setTaskLayout->addWidget(label_setTask);
  setTaskLayout->addWidget(missionComboBox_);
  setTaskLayout->addWidget(startLineEdit_);
  setTaskLayout->addWidget(goalLineEdit_);
  setTaskLayout->addWidget(button_setTask_);

  // --- buildMap section ---
  label_buildMap = new QLabel("Enter buildMap Data");
  label_buildMap->setAlignment(Qt::AlignCenter);
  label_buildMap->setFont(font);
  label_buildMap->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  le_mapCommand_ = new QLineEdit();
  le_mapCommand_->setPlaceholderText("Command (ex: 0,1,2,3) (int)");
  le_mapCommand_->setAlignment(Qt::AlignCenter);
  le_mapCommand_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  le_mapName_ = new QLineEdit();
  le_mapName_->setPlaceholderText("Map Name");
  le_mapName_->setAlignment(Qt::AlignCenter);
  le_mapName_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  le_mapId_ = new QLineEdit();
  le_mapId_->setPlaceholderText("Map ID");
  le_mapId_->setAlignment(Qt::AlignCenter);
  le_mapId_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  button_buildMap_ = new QPushButton("Publish buildMap");
  button_buildMap_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  auto buildMapLayout = new QVBoxLayout();
  buildMapLayout->setSpacing(5);
  buildMapLayout->addWidget(label_buildMap);
  buildMapLayout->addWidget(le_mapCommand_);
  buildMapLayout->addWidget(le_mapName_);
  buildMapLayout->addWidget(le_mapId_);
  buildMapLayout->addWidget(button_buildMap_);

  // Add the sections to the main layout.
  layout->addWidget(label_);
  layout->addLayout(setTaskLayout);
  layout->addLayout(buildMapLayout);

  // Connect signals and slots.
  QObject::connect(button_setTask_, &QPushButton::released, this, &TaskPanel::sendSetTask);
  QObject::connect(button_buildMap_, &QPushButton::released, this, &TaskPanel::sendBuildMap);
}

TaskPanel::~TaskPanel() = default;

void TaskPanel::onInitialize()
{
  // Access the ROS node and create the publisher for /vda5050/instantactions.
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>("/vda5050/instantactions", 10);
}

// Helper: Escape double quotes and backslashes for JSON strings.
QString TaskPanel::escapeJson(const QString & str) {
  QString escaped = str;
  escaped.replace("\\", "\\\\");
  escaped.replace("\"", "\\\"");
  return escaped;
}

// Construct and publish a setTask message.
void TaskPanel::sendSetTask()
{
  // Get mission from the combo box.
  QString mission = missionComboBox_->currentText();
  // Read start and goal from the input fields.
  QString startText = startLineEdit_->text();
  QString goalText  = goalLineEdit_->text();

  // Check if mission and goal fields are filled.
  if (mission.isEmpty() || goalText.isEmpty()){
      label_setTask->setText("Please fill in the required fields (mission, goal).");
      return;
  }
  
  // If start is empty, use default value 0.
  bool okStart = false;
  int startInt = startText.toInt(&okStart);
  QString startValue = startText.isEmpty() ? "null" : QString::number(startInt);

    
  // Convert goal to int.
  bool okGoal = false;
  int goalInt = goalText.toInt(&okGoal);
  if (!okGoal) {
      label_setTask->setText("Invalid number in goal node field.");
      return;
  }
  
  // Get current UTC time in ISO format with a trailing "Z".
  QString timestamp = QDateTime::currentDateTimeUtc().toString(Qt::ISODate) + "Z";
  
  // Construct the JSON string.
  // Note: mission remains a string; start and goal are output as numbers.
  QString innerJson = QString("{ \"headerId\": 1, \"timestamp\": \"%1\", \"version\": \"0.0.0\", "
                                "\"manufacturer\": \"CASELAB\", \"serialNumber\": \"SERIALNUMBER\", "
                                "\"actions\": [ { \"actionId\": \"setTask\", \"actionType\": \"setTask_ID\", "
                                "\"blockingType\": \"SOFT\", \"actionParameters\": [ { \"key\": \"signal\", "
                                "\"value\": [ \"1\", \"%2\", %3, %4 ] } ] } ] }")
                      .arg(timestamp)
                      .arg(mission)
                      .arg(startValue)
                      .arg(goalInt);
  
  std_msgs::msg::String msg;
  // Publish the inner JSON directly.
  msg.data = innerJson.toStdString();
  
  publisher_->publish(msg);
  label_setTask->setText("setTask message published.");
}

// Construct and publish a buildMap message.
void TaskPanel::sendBuildMap()
{
  QString commandText = le_mapCommand_->text();
  QString mapName    = le_mapName_->text();
  QString mapId      = le_mapId_->text();
  
  if (commandText.isEmpty()){
      label_buildMap->setText("Please fill in the Command field.");
      return;
  }
  
  bool okCommand = false;
  int commandInt = commandText.toInt(&okCommand);
  if (!okCommand) {
      label_buildMap->setText("Invalid number in Command field.");
      return;
  }
  
  // For command 3, mapName and mapId are required.
  if (commandInt == 3) {
      if (mapName.isEmpty()){
          label_buildMap->setText("For command 3, please fill in Map Name and Map ID.");
          return;
      }
  } else if (commandInt == 1 || commandInt == 2) {
      // For command 1,2, even if mapName and mapId are not needed,
      // send them as empty strings so that the JSON 'value' array always has three elements.
      mapName = "";
      mapId = "";
  } else {
      label_buildMap->setText("Unsupported command value.");
      return;
  }
  
  QString timestamp = QDateTime::currentDateTimeUtc().toString(Qt::ISODate) + "Z";
  
  QString innerJson;
  if (commandInt == 1 || commandInt == 2) {
      // When command is 1 or 2, include commandInt and two empty strings.
      innerJson = QString("{ \"headerId\": 1, \"timestamp\": \"%1\", \"version\": \"0.0.0\", "
                            "\"manufacturer\": \"CASELAB\", \"serialNumber\": \"SERIALNUMBER\", "
                            "\"actions\": [ { \"actionId\": \"buildMap\", \"actionType\": \"buildMap_ID\", "
                            "\"blockingType\": \"SOFT\", \"actionParameters\": [ { \"key\": \"signal\", "
                            "\"value\": [ %2, \"%3\", \"%4\" ] } ] } ] }")
                  .arg(timestamp)
                  .arg(commandInt)
                  .arg(mapName)
                  .arg(mapId);
  } else if (commandInt == 3) {
      // When command is 3, include commandInt, mapName, and mapId.
      innerJson = QString("{ \"headerId\": 1, \"timestamp\": \"%1\", \"version\": \"0.0.0\", "
                            "\"manufacturer\": \"CASELAB\", \"serialNumber\": \"SERIALNUMBER\", "
                            "\"actions\": [ { \"actionId\": \"buildMap\", \"actionType\": \"buildMap_ID\", "
                            "\"blockingType\": \"SOFT\", \"actionParameters\": [ { \"key\": \"signal\", "
                            "\"value\": [ %2, \"%3\", \"%4\" ] } ] } ] }")
                  .arg(timestamp)
                  .arg(commandInt)
                  .arg(mapName)
                  .arg(mapId);
  }
  
  std_msgs::msg::String msg;
  // Publish the inner JSON directly.
  msg.data = innerJson.toStdString();
  
  publisher_->publish(msg);
  label_buildMap->setText("buildMap message published.");
}


}  // namespace caselab_rviz_plugin

// Plugin export macro.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(caselab_rviz_plugin::TaskPanel, rviz_common::Panel)
