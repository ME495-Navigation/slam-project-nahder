// #include "catch_ros2/catch_ros2.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "nuturtle_control/srv/control.hpp"
// #include "geometry_msgs/msg/twist.hpp"

// using namespace std::chrono_literals;

// int count = 0;
// void vel_cb(const geometry_msgs::msg::Twist::SharedPtr)
// {
//   count = count + 1;
// }

// TEST_CASE("circle frequency", "[odom]") {

//   auto node = rclcpp::Node::make_shared("turtle_circle_test");

//   node->declare_parameter<double>("test_duration");
//   const auto TEST_DURATION =
//     node->get_parameter("test_duration").get_parameter_value().get<double>();

//   // Create a client and publisher
//   auto client = node->create_client<nuturtle_control::srv::Control>("~/control");
//   auto sub_cmd_vel = node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, &vel_cb);

//   auto request = std::make_shared<nuturtle_control::srv::Control::Request>();
//   request->velocity = 1.0;
//   request->radius = 0.5;

//   while (!client->wait_for_service(1s)) {
//     rclcpp::spin_some(node);
//   }

//   auto result = client->async_send_request(request);

//   while (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
//     rclcpp::spin_some(node);
//   }

//   rclcpp::Time start_time = rclcpp::Clock().now();
//   rclcpp::Time end_time = rclcpp::Clock().now();
//   while (
//     rclcpp::ok() &&
//     ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
//   )
//   {
//     end_time = rclcpp::Clock().now();
//   }

//   auto diff = end_time - start_time;
//   double hz = count / diff.seconds();

//   REQUIRE_THAT(hz, Catch::Matchers::WithinAbs(100.0, 10.0));
// }
