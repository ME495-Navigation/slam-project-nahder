#include "rclcpp/rclcpp.hpp"
#include "nuturtle_control/srv/control.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
public:
    Circle() : Node("circle")
    {
        control_srv = create_service<nuturtle_control::srv::Control>(
            "/control", std::bind(&Circle::control_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv;

    void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Received request to move in a circle with radius "
                                             << request->radius);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Circle>());
    rclcpp::shutdown();
    return 0;
}