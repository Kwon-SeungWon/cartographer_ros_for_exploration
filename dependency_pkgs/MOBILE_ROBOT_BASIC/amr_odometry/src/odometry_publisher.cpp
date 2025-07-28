#include "amr_odometry/odometry_publisher.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

namespace AMR{
    
Odom::Odom()
    : Node("to_odom")
{
    // Initialize broadcaster
    odom_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    setSubscriber();
    setParam();
    init();

    last_time = this->now().seconds(); // Initialize last_time
    
    // Initialize EKF
    initEKF();
}

Odom::~Odom()
{
    // No need to manually delete with unique_ptr
}

void Odom::setParam()
{
    robot_state = std::make_unique<RobotState>();
    robot_name = "";
    this->declare_parameter("enc_vel_gain", 1.0);
    this->declare_parameter("enc_wvel_gain", 1.0);
    this->declare_parameter("read_rate", 20);

    this->get_parameter("enc_vel_gain", enc_vel_gain);
    this->get_parameter("enc_wvel_gain", enc_wvel_gain);
    this->get_parameter("read_rate", read_rate);
}

void Odom::setSubscriber()
{
    enc_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("enc_vel", 10, std::bind(&Odom::encVelCallback, this, std::placeholders::_1));
}

void Odom::init()
{
    loop_rate = std::make_unique<rclcpp::Rate>(read_rate);
    
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
}

void Odom::initEKF()
{
    // State vector: [x, y, yaw, v, w]
    x_ = Eigen::VectorXd::Zero(5);
    
    // Covariance matrix
    P_ = Eigen::MatrixXd::Identity(5, 5);
    P_ *= 0.1; // Initial uncertainty
    
    // Process noise covariance
    Q_ = Eigen::MatrixXd::Identity(5, 5);
    Q_(0,0) = 0.1; // x position noise
    Q_(1,1) = 0.1; // y position noise
    Q_(2,2) = 0.1; // yaw noise
    Q_(3,3) = 0.1; // linear velocity noise
    Q_(4,4) = 0.1; // angular velocity noise
    
    // Measurement noise covariance
    R_ = Eigen::MatrixXd::Identity(3, 3);
    R_(0,0) = 0.1; // x position measurement noise
    R_(1,1) = 0.1; // y position measurement noise
    R_(2,2) = 0.1; // yaw measurement noise
}

void Odom::predictEKF(double dt)
{
    // State transition matrix
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    F(0,3) = dt * cos(x_(2)); // x = x + v*cos(yaw)*dt
    F(1,3) = dt * sin(x_(2)); // y = y + v*sin(yaw)*dt
    F(2,4) = dt;              // yaw = yaw + w*dt
    
    // Predict state
    x_ = F * x_;
    
    // Predict covariance
    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void Odom::updateEKF()
{
    // Measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 5);
    H(0,0) = 1.0; // x position
    H(1,1) = 1.0; // y position
    H(2,2) = 1.0; // yaw
    
    // Measurement vector
    Eigen::VectorXd z(3);
    z << robot_state->x, robot_state->y, robot_state->yaw;
    
    // Innovation
    Eigen::VectorXd y = z - H * x_;
    
    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state
    x_ = x_ + K * y;
    
    // Update covariance
    P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * P_;
}

void Odom::encVelCallback(const geometry_msgs::msg::Twist::SharedPtr msgs)
{
    std::lock_guard<std::mutex> lock(enc_vel_mutex);
    enc_vel = *msgs;

    rclcpp::Time odom_current_time = this->now();
    current_time = odom_current_time.seconds();
    dt = current_time - last_time;

    if (dt <= 0) return;

    enc_vel.linear.x *= enc_vel_gain;
    enc_vel.angular.z *= enc_wvel_gain;
    
    // Update EKF state with velocity measurements
    x_(3) = enc_vel.linear.x;  // linear velocity
    x_(4) = enc_vel.angular.z; // angular velocity
    
    // Predict step
    predictEKF(dt);
    
    // Update step
    updateEKF();
    
    // Update robot state with EKF estimates
    robot_state->x = x_(0);
    robot_state->y = x_(1);
    robot_state->yaw = x_(2);
    
    // Calculate odometry
    dx = (enc_vel.linear.x * cos(robot_state->yaw) - enc_vel.linear.y * sin(robot_state->yaw)) * dt;
    dy = (enc_vel.linear.x * sin(robot_state->yaw) + enc_vel.linear.y * cos(robot_state->yaw)) * dt;
    dyaw = enc_vel.angular.z * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, robot_state->yaw);
    q.normalize();
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    odom.header.stamp = odom_current_time;
    odom.pose.pose.position.x = robot_state->x;
    odom.pose.pose.position.y = robot_state->y;
    odom.pose.pose.orientation = odom_quat;

    // Update covariance in odometry message
    for(int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = P_(0,0);  // x position variance
    odom.pose.covariance[7] = P_(1,1);  // y position variance
    odom.pose.covariance[35] = P_(2,2); // yaw variance

    odom.twist.twist = enc_vel;

    odom_tf.header.stamp = odom_current_time;
    odom_tf.transform.translation.x = robot_state->x;
    odom_tf.transform.translation.y = robot_state->y;
    odom_tf.transform.rotation = odom_quat;

    odom_pub_->publish(odom);
    odom_tf_broadcaster_->sendTransform(odom_tf);

    last_time = current_time;
}

} // namespace AMR

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AMR::Odom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
