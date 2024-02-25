#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <stdexcept>

namespace turtlelib
{

DiffDrive::DiffDrive()
: wheel_radius(0.033), track_width(0.16) {}

DiffDrive::DiffDrive(double wheel_radius, double track_width)
: wheel_radius(wheel_radius), track_width(track_width) {}

void DiffDrive::forwardKinematics(const wheelVel u)
{
    Twist2D delta_q, Vb; // just initialize these directly e.g. const auto Vb = computeBodyTwist(u)

  // convert wheel configuration to body twist
  Vb = computeBodyTwist(u);

  // integrate the body twist to get the new configuration
  Transform2D T_b_bp = integrate_twist(Vb); // const auto

  // get the robot location in the world frame
  Transform2D T_wb{Vector2D{cur_config.x, cur_config.y}, cur_config.theta};

  // get the new robot location in the world frame
  Transform2D T_w_bp = T_wb * T_b_bp;

  // update the robot configuration
  set_config({T_w_bp.translation().x, T_w_bp.translation().y, normalize_angle(T_w_bp.rotation())});
}

Twist2D DiffDrive::computeBodyTwist(const wheelVel u) const
{
    Twist2D twist; // initialize this directly
  double d{0.5 * track_width}; // const auto

  twist.omega = (wheel_radius / (2 * d)) * (u.right_wheel_vel - u.left_wheel_vel); // 2.0

  twist.x = (wheel_radius / 2) * (u.right_wheel_vel + u.left_wheel_vel);

  twist.y = 0.0;

  return twist;
}

wheelVel DiffDrive::inverseKinematics(const Twist2D twist) const
{
    wheelVel u; // initialzie directly
  double d{0.5 * track_width};

  if (twist.y != 0.0) {
    throw std::logic_error("y component of twist must be zero");
  } else {
    u.right_wheel_vel = (1 / wheel_radius) * (-d * twist.omega + twist.x);
    u.left_wheel_vel = (1 / wheel_radius) * (d * twist.omega + twist.x);
    return u;
  }
}

}
