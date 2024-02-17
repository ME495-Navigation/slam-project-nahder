#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{

/// @brief robot configuration
struct robotConfig
{
/// @brief x position
  double x{0.0};

/// @brief y position
  double y{0.0};

/// @brief orientation
  double theta{0.0};
};

/// @brief wheel velocities (can also represent configuration)
struct wheelVel
{
/// @brief left wheel velocity
  double left_wheel_vel{0.0};

/// @brief right wheel velocity
  double right_wheel_vel{0.0};
};

/// @brief differential drive robot kinematics
class DiffDrive
{

public:
  /// @brief diff drive constructor
  /// @param wheel_radius
  /// @param track_width
  DiffDrive(double wheel_radius, double track_width);

  /// @brief default constructor (turtlebot3 specific values)
  DiffDrive();

  /// @brief forward kinematics. update robot configuration given wheel motion
  /// @param new_wheel_config {left_wheel_vel, right_wheel_vel}
  void forwardKinematics(const wheelVel new_wheel_config);

  /// @brief inverse kinematics. compute the wheel velocities required to achieve a desired twist
  /// @param twist
  /// @return wheelVel object with left and right wheel velocities
  wheelVel inverseKinematics(const Twist2D twist) const;

  /// @brief convert wheel velocities to actual body twist
  /// @param u wheelVel object with left and right wheel velocities
  /// @return Twist2D object of {omega, xDot, yDot}
  Twist2D computeBodyTwist(const wheelVel u) const;

  /// @brief getter for robot configuration
  /// @return robotConfig object of {x,y,theta}
  robotConfig get_config() const {return cur_config;}

  /// @brief setter for robot configuration
  /// @param config
  void set_config(robotConfig config) {cur_config = config;}

private:
  robotConfig cur_config;
  wheelVel cur_wheel_vel;

  double wheel_radius;
  double track_width;
};

}
