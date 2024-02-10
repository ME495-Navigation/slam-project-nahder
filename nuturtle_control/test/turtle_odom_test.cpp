#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

using namespace std::chrono_literals;

auto srv_found{false};
auto xform_found{false};

TEST_CASE("Initial pose service", "[odometry]") {
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");
  srv_found = false;

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  auto client = node->create_client<nuturtle_control::srv::InitialPose>("initial_pose");

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    if (client->wait_for_service(0s)) {
      srv_found = true;

      auto request = std::make_shared<nuturtle_control::srv::InitialPose::Request>();
      request->x = 1.0;
      request->y = 2.0;
      request->theta = 0.5;
      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->success) {
          try {
            auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
            tf2_ros::TransformListener tf_listener(*tf_buffer);
            auto t = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            xform_found = true;
            REQUIRE_THAT(t.transform.translation.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
            REQUIRE_THAT(t.transform.translation.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
            REQUIRE_THAT(t.transform.rotation.z, Catch::Matchers::WithinAbs(0.5, 1e-5));
          } catch (const tf2::TransformException & ex) {
            return;
          }
        }
      }
      break;
    }
    rclcpp::spin_some(node);
  }
  CHECK(srv_found);
  CHECK(xform_found);
}

TEST_CASE("odom->base_footprint transform", "[odometry]") {
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");

  node->declare_parameter<double>("test_duration");
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::Time start_time = rclcpp::Clock().now();

  while (rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION)))
  {
    try {
      auto t = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      xform_found = true;
    } catch (const tf2::TransformException & ex) {
      return;
    }
    rclcpp::spin_some(node);
  }
  CHECK(xform_found);
}
