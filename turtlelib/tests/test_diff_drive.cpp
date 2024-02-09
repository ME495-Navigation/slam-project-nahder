#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <iostream>

namespace turtlelib
{

TEST_CASE("Error checking twist", "[diff drive]")
{
  double wheelRadius{0.12};
  double trackWidth{0.12};

  DiffDrive robot{wheelRadius, trackWidth};

  Twist2D desiredTwist{0.1, 0.2, 1.5};       // twist with a y component, which is invalid

  bool caughtException = false;
  try {
    robot.inverseKinematics(desiredTwist);
  } catch (const std::logic_error & e) {
    caughtException = true;
    std::string expectedMessage = "y component of twist must be zero for non-slip conditions";
    REQUIRE(std::string(e.what()) == expectedMessage);
  }

  REQUIRE(caughtException);
}
// robot drives forward (both FK and IK)
TEST_CASE("FK, pure forward", "[diff drive]")
{
  DiffDrive robot{0.1, 0.2};       // wheelRadius, trackWidth
  wheelVel u{1.0, 1.0};            // leftWheelVel, rightWheelVel

  robot.forwardKinematics(u);
  robotConfig config = robot.get_config();

  REQUIRE_THAT(config.x, Catch::Matchers::WithinAbs(0.1, 1e-5));
  REQUIRE_THAT(config.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(config.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("IK, pure forward", "[diff drive]")
{
  DiffDrive robot{0.1, 0.2};
  Twist2D desiredTwist{0.0, 0.2, 0.0};

  // wheel velocities to achieve desiredTwist
  wheelVel u = robot.inverseKinematics(desiredTwist);

  // phiDot = v / r
  double expectedWheelVelocity = desiredTwist.x / 0.1;

  REQUIRE_THAT(u.right_wheel_vel, Catch::Matchers::WithinAbs(expectedWheelVelocity, 1e-5));
  REQUIRE_THAT(u.left_wheel_vel, Catch::Matchers::WithinAbs(expectedWheelVelocity, 1e-5));
}

TEST_CASE("FK, pure rotation", "[diff drive]")
{
  double w_r{0.1};
  double track{0.35};
  DiffDrive robot{w_r, track};
  wheelVel u{-1.0, 1.0};

  robot.forwardKinematics(u);
  robotConfig config = robot.get_config();

  // no translation expected, only rotation
  REQUIRE_THAT(config.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
  REQUIRE_THAT(config.y, Catch::Matchers::WithinAbs(0.0, 1e-5));

  double expectedThetaChange = (w_r / track) * (u.right_wheel_vel - u.left_wheel_vel);
  REQUIRE_THAT(config.theta, Catch::Matchers::WithinAbs(expectedThetaChange, 1e-5));
}
TEST_CASE("IK, pure rotation", "[diff drive]")
{
  double wheelRadius{1.5};
  double trackWidth{0.8};
  double d{trackWidth / 2.0};

  DiffDrive robot{wheelRadius, trackWidth};

  Twist2D desiredTwist{1.0, 0.0, 0.0};

  wheelVel u = robot.inverseKinematics(desiredTwist);

  double expectedRightWheelVel = (desiredTwist.x - d * desiredTwist.omega) / wheelRadius;
  double expectedLeftWheelVel = (desiredTwist.x + d * desiredTwist.omega) / wheelRadius;

  REQUIRE_THAT(u.right_wheel_vel, Catch::Matchers::WithinAbs(expectedRightWheelVel, 1e-5));
  REQUIRE_THAT(u.left_wheel_vel, Catch::Matchers::WithinAbs(expectedLeftWheelVel, 1e-5));
}

TEST_CASE("ForwardKinematics(), Rotation and Translation", "[diff_drive]")
{
  DiffDrive robot{1.0, 4.0};
  wheelVel u{1.2, 3.4};

  robot.forwardKinematics(u);
  REQUIRE_THAT(robot.get_config().x, Catch::Matchers::WithinAbs(2.1857829573, 1e-5));
  REQUIRE_THAT(robot.get_config().y, Catch::Matchers::WithinAbs(0.616715635, 1e-5));
  REQUIRE_THAT(robot.get_config().theta, Catch::Matchers::WithinAbs(0.55, 1e-5));
}

TEST_CASE("IK, follows the arc of a circle", "[diff drive]")
{
  double wheelRadius{0.12};
  double trackWidth{0.12};
  double d{trackWidth / 2.0};

  DiffDrive robot{wheelRadius, trackWidth};

  Twist2D desiredTwist{0.1, 0.2, 0.0};

  wheelVel u = robot.inverseKinematics(desiredTwist);

  double expectedRightWheelVel = (desiredTwist.x - d * desiredTwist.omega) / wheelRadius;
  double expectedLeftWheelVel = (desiredTwist.x + d * desiredTwist.omega) / wheelRadius;

  REQUIRE_THAT(u.right_wheel_vel, Catch::Matchers::WithinAbs(expectedRightWheelVel, 1e-5));
  REQUIRE_THAT(u.left_wheel_vel, Catch::Matchers::WithinAbs(expectedLeftWheelVel, 1e-5));
}

}
