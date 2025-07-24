#ifndef VELOCITY_PROFILE_HPP
#define VELOCITY_PROFILE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <cmath>

namespace AMR {

class VelocityProfile {
private:
  std::shared_ptr<geometry_msgs::msg::Twist> target_speed;
  std::shared_ptr<geometry_msgs::msg::Twist> current_speed;
  double control_hz;
  geometry_msgs::msg::Twist acc;
  double tolerance;

  // Internal method for velocity calculation
  double calcVel(double before_speed, double acc);

public:
  // Constructor & Destructor
  VelocityProfile(geometry_msgs::msg::Twist acc, double control_hz = 10.0);
  ~VelocityProfile();

  // Methods
  void resetSpeed(geometry_msgs::msg::Twist init_speed);
  geometry_msgs::msg::Twist calc(geometry_msgs::msg::Twist vel);
  bool checkAcceleration(geometry_msgs::msg::Twist current_vel,
                         geometry_msgs::msg::Twist target_vel);
  geometry_msgs::msg::Twist reduceVelocity(geometry_msgs::msg::Twist current_vel, double ratio = 0.5);
};

}  // namespace amr

#endif  // VELOCITY_PROFILE_HPP
