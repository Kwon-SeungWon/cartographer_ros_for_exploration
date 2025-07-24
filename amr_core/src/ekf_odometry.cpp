#include "amr_core/ekf_odometry.h"

namespace AMR {

EKFOdometry::EKFOdometry()
{
    init();
}

EKFOdometry::~EKFOdometry()
{
}

void EKFOdometry::init()
{
    // State vector: [x, y, yaw, v, w]
    x_ = Eigen::VectorXd::Zero(5);
    
    // Covariance matrix
    P_ = Eigen::MatrixXd::Identity(5, 5);
    P_ *= 0.01; // Initial uncertainty
    
    // Process noise covariance
    Q_ = Eigen::MatrixXd::Identity(5, 5);
    Q_(0,0) = 0.01; // x position noise
    Q_(1,1) = 0.01; // y position noise
    Q_(2,2) = 0.01; // yaw noise
    Q_(3,3) = 0.05; // linear velocity noise
    Q_(4,4) = 0.05; // angular velocity noise
    
    // Measurement noise covariance
    R_ = Eigen::MatrixXd::Identity(3, 3);
    R_(0,0) = 0.01; // x position measurement noise
    R_(1,1) = 0.01; // y position measurement noise
    R_(2,2) = 0.01; // yaw measurement noise
}

void EKFOdometry::predict(double dt)
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

void EKFOdometry::update(double x, double y, double yaw)
{
    // Measurement matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 5);
    H(0,0) = 1.0; // x position
    H(1,1) = 1.0; // y position
    H(2,2) = 1.0; // yaw
    
    // Measurement vector
    Eigen::VectorXd z(3);
    z << x, y, yaw;
    
    // Innovation
    Eigen::VectorXd y_innov = z - H * x_;
    
    // Innovation covariance
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
    
    // Kalman gain
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Update state
    x_ = x_ + K * y_innov;
    
    // Update covariance
    P_ = (Eigen::MatrixXd::Identity(5, 5) - K * H) * P_;
}

void EKFOdometry::setVelocity(double v, double w)
{
    x_(3) = v;  // linear velocity
    x_(4) = w;  // angular velocity
}

} // namespace AMR 