#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;


TEST_CASE("test 1=1", "[turtle_control]") {
  REQUIRE_THAT(1, Catch::Matchers::WithinAbs(1, 0.0001));
}
