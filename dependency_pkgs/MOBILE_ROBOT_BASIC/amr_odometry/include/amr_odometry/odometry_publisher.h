#ifndef __AMR_ODOMETRY_H__
#define __AMR_ODOMETRY_H__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <mutex>
#include <memory>
#include <Eigen/Dense>

namespace AMR
{

struct RobotState
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class Odom : public rclcpp::Node
{
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr enc_vel_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

  int read_rate;
  std::unique_ptr<rclcpp::Rate> loop_rate;
  
  geometry_msgs::msg::Twist enc_vel;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped odom_tf;

  std::string robot_name;

  double current_time, last_time;
  double dx, dy, dyaw, dt;
  double enc_vel_gain, enc_wvel_gain;
  std::unique_ptr<RobotState> robot_state;
  bool use_ekf;
  void init();
  void setParam();
  void encVelCallback(const geometry_msgs::msg::Twist::SharedPtr msgs);
  void setSubscriber();
  std::mutex enc_vel_mutex;

  // EKF variables
  Eigen::VectorXd x_;  // State vector [x, y, yaw, v, w]
  Eigen::MatrixXd P_;  // Covariance matrix
  Eigen::MatrixXd Q_;  // Process noise covariance
  Eigen::MatrixXd R_;  // Measurement noise covariance

public:
  Odom();
  ~Odom();

  void initEKF();
  void predictEKF(double dt);
  void updateEKF();
  void publishOdometry();
  void updateBasicOdometry(double dt);
  void updateEKFOdometry(double dt);
};

}  // namespace AMR

#endif
