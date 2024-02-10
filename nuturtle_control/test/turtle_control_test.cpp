#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;

//turtle_control is running
//turtle control:
//  - subscribes to cmd_vel (tw) --> publishes wheel_cmd (MCU)
//  - subscribes to sensor_data (ticks) --> publishes joint_states (radians)

// Write a test that verifies that cmd_vel commands with pure translation result in the appropriate
// wheel_cmd being published.

// Write a test that verifies that cmd_vel commands with pure rotation in the appropriate wheel_cmd


// Write a test that verifies that encoder data on sensors is converted to joint_states properly


auto left_wheel_vel{0.0}, right_wheel_vel{0.0}, left_joint_pos{0.0}, right_joint_pos{0.0},
left_joint_vel{0.0}, right_joint_vel{0.0};

bool sub_found{false};

void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands msg)
{
  left_wheel_vel = msg.left_velocity;
  right_wheel_vel = msg.right_velocity;
}

void joint_states_callback(const sensor_msgs::msg::JointState msg)
{
  left_joint_pos = msg.position[0];
  right_joint_pos = msg.position[1];
  left_joint_vel = msg.velocity[0];
  right_joint_vel = msg.velocity[1];
}

//Testing the conversion from cmd_vel (desired body twist) to wheel_cmd (MCU)
TEST_CASE("Pure translation", "[controller]")
{
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = 0.1;

  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &wheel_cmd_callback);

  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    cmd_vel_pub->publish(cmd_vel_msg);
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(left_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
  REQUIRE_THAT(right_wheel_vel, Catch::Matchers::WithinAbs(126.0, 1e-5));
}
