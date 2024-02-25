// comments?
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    declare_parameter("frequency", 100);
    frequency = get_parameter("frequency").as_int();

    control_srv = create_service<nuturtle_control::srv::Control>(
      "/control", std::bind(
        &Circle::control_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    reverse_srv = create_service<std_srvs::srv::Empty>(
      "/reverse", std::bind(
        &Circle::reverse_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    stop_srv = create_service<std_srvs::srv::Empty>(
      "/stop", std::bind(
        &Circle::stop_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    timer = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency),
      std::bind(&Circle::timer_callback, this));

    cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::TimerBase::SharedPtr timer;
  geometry_msgs::msg::Twist body_twist;

  int frequency;
  bool stopped{true};   // initialize as true to not publish until a control command is received

  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    // based on v = w * r, so w = v / r. (the velocity parameter is the angular in rad/s)
    auto linear_velocity{request->velocity * request->radius};
    body_twist.linear.x = linear_velocity;
    body_twist.angular.z = request->velocity;
    stopped = false;

    RCLCPP_INFO_STREAM(
      get_logger(), "Received request to move in a circle with radius "
        << request->radius << " and angular velocity " << request->velocity);
  }

  void reverse_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    body_twist.linear.x *= -1;
    body_twist.angular.z *= -1;
    body_twist.linear.y = 0;

    RCLCPP_INFO(get_logger(), "Received request to reverse direction");
  }

  void stop_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(get_logger(), "Received request to stop");
    body_twist.linear.x = 0;
    body_twist.linear.y = 0;
    body_twist.angular.z = 0;
    stopped = true;
    cmd_pub->publish(body_twist);     // publish a single cmd_vel command of zero when stopped
  }

  void timer_callback()
  {
    // only publish if not stopped
    if (!stopped) {
      cmd_pub->publish(body_twist);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
