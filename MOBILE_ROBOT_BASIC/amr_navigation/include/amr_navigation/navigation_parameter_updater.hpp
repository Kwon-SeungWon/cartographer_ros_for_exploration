#ifndef NAVIGATION_PARAMETER_UPDATER_HPP
#define NAVIGATION_PARAMETER_UPDATER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <cmath>

/**
 * @brief NavigationParameterUpdater obtains the current robot pose via TF2 lookup and the goal pose from Nav2.
 *
 * It periodically checks (via a timer callback) the transform from "map" to "base_link" to obtain the robot's
 * current pose. It also subscribes to the goal pose topic (e.g., "/goal_pose"). When the Euclidean distance
 * between the current pose (from TF2) and the goal is below a specified threshold, the node calls the
 * "update_params" service once.
 */
class NavigationParameterUpdater : public rclcpp::Node
{
public:
  /**
   * @brief Constructor: sets up TF2 lookup, the goal pose subscriber, and the update service client.
   */
  NavigationParameterUpdater();

private:
  /**
   * @brief Callback for receiving the goal pose from Nav2.
   *
   * Stores the new goal and resets the service-called flag so that a parameter update can be triggered.
   *
   * @param msg Pointer to the received PoseStamped message representing the goal.
   */
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Timer callback that uses TF2 to update the current robot pose.
   *
   * Looks up the transform from "map" to "base_link" and stores it as the current pose, then checks
   * if the distance to the goal is within the threshold.
   */
  void updateCurrentPose();

  /**
   * @brief Checks the distance between the current pose (obtained via TF2) and the goal pose.
   *
   * If the robot is within the threshold distance and the service has not yet been called for this goal,
   * it triggers the update service.
   */
  void checkAndUpdateParameters();

  /**
   * @brief Calls the "update_params" service to update parameters.
   *
   * Waits for the service to be available, sends a request, and logs the result.
   */
  void callUpdateService();

  /**
   * @brief Determines whether a goal has been received.
   *
   * A goal is considered received if the header timestamp is non-zero.
   *
   * @return true if a goal has been received, false otherwise.
   */
  bool goalReceived();

  // Parameters
  double threshold_distance_;  ///< Threshold distance (in meters) to trigger the parameter update.
  bool service_called_;        ///< Flag to ensure the service is called only once per goal.

  // Latest poses
  geometry_msgs::msg::PoseStamped current_pose_;  ///< Current robot pose (obtained via TF2).
  geometry_msgs::msg::PoseStamped goal_pose_;     ///< Latest goal pose (from Nav2).

  // ROS 2 communication objects
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;  ///< Subscriber for the goal pose.
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr update_client_;                ///< Client for the update_params service.
  rclcpp::TimerBase::SharedPtr timer_;                                             ///< Timer for periodically updating the current pose.

  // TF2 objects for looking up the robot's current pose.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // NAVIGATION_PARAMETER_UPDATER_HPP
