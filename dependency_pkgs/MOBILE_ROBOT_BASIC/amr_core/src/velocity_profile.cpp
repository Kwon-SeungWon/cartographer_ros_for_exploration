#include "amr_core/velocity_profile.h"

namespace AMR {

VelocityProfile::VelocityProfile(geometry_msgs::msg::Twist acc, double control_hz)
  : acc(acc), control_hz(control_hz),
    target_speed(std::make_shared<geometry_msgs::msg::Twist>()),
    current_speed(std::make_shared<geometry_msgs::msg::Twist>())
{
}

VelocityProfile::~VelocityProfile()
{
  target_speed.reset();
  current_speed.reset();
}

void VelocityProfile::resetSpeed(geometry_msgs::msg::Twist init_speed)
{
  *current_speed = init_speed;
}

double VelocityProfile::calcVel(double current_speed, double acc)
{
    return current_speed + (acc * (1.0 / control_hz));
}

geometry_msgs::msg::Twist VelocityProfile::calc(geometry_msgs::msg::Twist vel)
{
    *target_speed = vel;
    double dt = (1.0 / control_hz);
    double check_speed = target_speed->linear.x - current_speed->linear.x;
    if (fabs(check_speed) < acc.linear.x * dt) {
        current_speed->linear.x = target_speed->linear.x;
    } else {
        current_speed->linear.x = calcVel(current_speed->linear.x, std::copysign(acc.linear.x, check_speed));
    }

    check_speed = target_speed->linear.y - current_speed->linear.y;
    if (fabs(check_speed) < acc.linear.y * dt) {
        current_speed->linear.y = target_speed->linear.y;
    } else {
        current_speed->linear.y = calcVel(current_speed->linear.y, std::copysign(acc.linear.y, check_speed));
    }

    check_speed = target_speed->angular.z - current_speed->angular.z;
    if (fabs(check_speed) < acc.angular.z * dt) {
        current_speed->angular.z = target_speed->angular.z;
    } else {
        current_speed->angular.z = calcVel(current_speed->angular.z, std::copysign(acc.angular.z, check_speed));
    }

    return *current_speed;
}

bool VelocityProfile::checkAcceleration(geometry_msgs::msg::Twist current_speed,
                                        geometry_msgs::msg::Twist target_speed)
{
  bool result = true;
  double check_acc = target_speed.linear.x - current_speed.linear.x;
  result = result && (fabs(check_acc) <= acc.linear.x);
  check_acc = target_speed.linear.y - current_speed.linear.y;
  result = result && (fabs(check_acc) <= acc.linear.y);
  check_acc = target_speed.angular.z - current_speed.angular.z;
  result = result && (fabs(check_acc) <= acc.angular.z);
  return result;
}

geometry_msgs::msg::Twist VelocityProfile::reduceVelocity(geometry_msgs::msg::Twist current_vel, double ratio)
{
  geometry_msgs::msg::Twist reduced_vel;
  reduced_vel.linear.x = current_vel.linear.x * ratio;
  reduced_vel.linear.y = current_vel.linear.y * ratio;
  reduced_vel.angular.z = current_vel.angular.z * ratio;
  return reduced_vel;
}

}  // namespace amr
