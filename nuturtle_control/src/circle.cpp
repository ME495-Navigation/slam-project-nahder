#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
    Circle() : Node("circle")
    {
        declare_parameter("frequency", 100);
        frequency = get_parameter("frequency").as_int();

        control_srv = create_service<nuturtle_control::srv::Control>(
            "/control", std::bind(&Circle::control_callback, this, std::placeholders::_1,
                                  std::placeholders::_2));

        reverse_srv = create_service<std_srvs::srv::Empty>(
            "/reverse", std::bind(&Circle::reverse_callback, this, std::placeholders::_1,
                                  std::placeholders::_2));

        stop_srv = create_service<std_srvs::srv::Empty>(
            "/stop", std::bind(&Circle::stop_callback, this, std::placeholders::_1,
                               std::placeholders::_2));

        timer = create_wall_timer(100ms, std::bind(&Circle::timer_callback, this));

        cmd_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::TimerBase::SharedPtr timer; 
    int frequency;

    void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
                          std::shared_ptr<nuturtle_control::srv::Control::Response>)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Received request to move in a circle with radius "
                                             << request->radius);
        // TODO
    }

    void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(get_logger(), "Received request to reverse direction");
        // TODO
    }

    void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                       std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        RCLCPP_INFO(get_logger(), "Received request to stop");
        // TODO
    }

    void timer_callback()
    {
        //publish cmd_vel commands at a fixed rate
        // TODO
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Circle>());
    rclcpp::shutdown();
    return 0;
}