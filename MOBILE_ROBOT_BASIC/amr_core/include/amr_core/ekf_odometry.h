#ifndef EKF_ODOMETRY_H
#define EKF_ODOMETRY_H

#include <Eigen/Dense>
#include <memory>

namespace AMR {

class EKFOdometry {
public:
    EKFOdometry();
    ~EKFOdometry();

    void init();
    void predict(double dt);
    void update(double x, double y, double yaw);
    void setVelocity(double v, double w);
    
    // Get state estimates
    double getX() const { return x_(0); }
    double getY() const { return x_(1); }
    double getYaw() const { return x_(2); }
    
    // Get covariance
    double getCovariance(int i, int j) const { return P_(i,j); }

private:
    // State vector: [x, y, yaw, v, w]
    Eigen::VectorXd x_;
    
    // Covariance matrix
    Eigen::MatrixXd P_;
    
    // Process noise covariance
    Eigen::MatrixXd Q_;
    
    // Measurement noise covariance
    Eigen::MatrixXd R_;
};

} // namespace AMR

#endif // EKF_ODOMETRY_H 