#include "amr_navigation/navigation_parameter_updater.hpp"

NavigationParameterUpdater::NavigationParameterUpdater()
: Node("navigation_parameter_updater"), service_called_(false)
{
  // Declare the threshold distance parameter (default: 1.0 meter)
  this->declare_parameter<double>("threshold_distance", 1.0);
  threshold_distance_ = this->get_parameter("threshold_distance").as_double();

  // Initialize the TF2 buffer and listener to obtain current pose via TF lookup.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer to periodically update the current pose using TF (e.g., every 100 ms).
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NavigationParameterUpdater::updateCurrentPose, this));

  // Subscription to the goal pose published by Nav2 (e.g., "/goal_pose")
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&NavigationParameterUpdater::goalPoseCallback, this, std::placeholders::_1));

  // Create a client for the "update_params" service provided by the ParameterUpdater node.
  update_client_ = this->create_client<std_srvs::srv::Trigger>("update_params");

  RCLCPP_INFO(this->get_logger(), "NavigationParameterUpdater node has started.");
}

void NavigationParameterUpdater::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // Store the new goal pose and log its coordinates.
  goal_pose_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Received new goal: [%.2f, %.2f]",
              goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  // Reset the flag so that the service can be called for this new goal.
  service_called_ = false;
}

void NavigationParameterUpdater::updateCurrentPose()
{
  // Attempt to lookup the transform from "map" to "base_link".
  geometry_msgs::msg::TransformStamped map_to_base;
  try {
    map_to_base = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    // Convert the transform into a PoseStamped message.
    current_pose_.header = map_to_base.header;
    current_pose_.pose.position.x = map_to_base.transform.translation.x;
    current_pose_.pose.position.y = map_to_base.transform.translation.y;
    current_pose_.pose.position.z = map_to_base.transform.translation.z;
    current_pose_.pose.orientation = map_to_base.transform.rotation;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform from map to base_link: %s", ex.what());
    return;
  }

  // After updating the current pose, check if we should update parameters.
  checkAndUpdateParameters();
}

void NavigationParameterUpdater::checkAndUpdateParameters()
{
  // If a goal has not been received yet, do nothing.
  if (!goalReceived()) {
    return;
  }

  // Compute the Euclidean distance between current pose and goal pose.
  double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
  double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(this->get_logger(), "Distance to goal: %.2f m", distance);

  // If within the threshold distance and the service has not yet been called, call the update service.
  if (distance <= threshold_distance_ && !service_called_) {
    RCLCPP_INFO(this->get_logger(), "Within threshold distance. Calling update_params service...");
    callUpdateService();
    service_called_ = true;
  }
}

bool NavigationParameterUpdater::goalReceived()
{
  // A goal is considered received if its header stamp is non-zero.
  return (goal_pose_.header.stamp.sec != 0 || goal_pose_.header.stamp.nanosec != 0);
}

void NavigationParameterUpdater::callUpdateService()
{
  // Wait for the "update_params" service to become available.
  if (!update_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "The update_params service is not available.");
    return;
  }

  // Create a request for the Trigger service.
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the request asynchronously.
  auto future_result = update_client_->async_send_request(request);

  // Block until the service call completes.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result)
      == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future_result.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Parameters updated successfully: %s", response->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter update failed: %s", response->message.c_str());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call the update_params service.");
  }
}

