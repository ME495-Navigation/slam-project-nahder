#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"


using namespace std::chrono_literals;

auto left_wheel_vel{0.0}, right_wheel_vel{0.0}, left_joint_pos{0.0}, right_joint_pos{0.0},
left_joint_vel{0.0}, right_joint_vel{0.0};

auto sub_found{false};

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


//Testing the conversion from cmd_vel (desired body twist) to wheel_cmd (MCU) for pure translation
TEST_CASE("Pure translation", "[controller]")
{
  sub_found = false;
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

//Testing the conversion from cmd_vel (desired body twist) to wheel_cmd (MCU) for pure rotation
TEST_CASE("Pure rotation", "[controller]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = 0.25;

  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_sub = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    "wheel_cmd", 10, &wheel_cmd_callback);

  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    cmd_vel_pub->publish(cmd_vel_msg);
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(left_wheel_vel, Catch::Matchers::WithinAbs(25.0, 1e-5));
  REQUIRE_THAT(right_wheel_vel, Catch::Matchers::WithinAbs(-25.0, 1e-5));
}

// Testing the conversion from sensor_data (ticks) to joint_states (radians)
TEST_CASE("Ticks to radians", "[controller]")
{
  sub_found = false;
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  rclcpp::Time start_time = rclcpp::Clock().now();

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  nuturtlebot_msgs::msg::SensorData sensor_data_msg;
  sensor_data_msg.left_encoder = 100;
  sensor_data_msg.right_encoder = 100;

  auto sensor_data_pub = node->create_publisher<nuturtlebot_msgs::msg::SensorData>(
    "sensor_data", 10);
  auto joint_states_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, &joint_states_callback);

  while (rclcpp::ok() && !(sub_found) &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    sensor_data_pub->publish(sensor_data_msg);
    rclcpp::spin_some(node);
  }
  REQUIRE_THAT(left_joint_pos, Catch::Matchers::WithinAbs(0.15339, 1e-5));
  REQUIRE_THAT(right_joint_pos, Catch::Matchers::WithinAbs(0.15339, 1e-5));
}
