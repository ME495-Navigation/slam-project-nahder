#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
// https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node
using namespace std::chrono_literals;

class NUSim : public rclcpp::Node
{

public:
    NUSim() : Node("nusim")
    { 
        timestep = 0;
        timestep_pub = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        reset_srv = create_service<std_srvs::srv::Trigger>("~/reset",
                                                           std::bind(&NUSim::reset_callback,
                                                                     this, std::placeholders::_1,
                                                                     std::placeholders::_2));

        declare_parameter("rate", 200.0);
        rate = get_parameter("rate").as_double();
        timer = create_wall_timer(1s / rate, std::bind(&NUSim::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;

    double rate;
    uint64_t timestep;

    void timer_callback()
    {
        std_msgs::msg::UInt64 msg;

        timestep++;
        msg.data = timestep;
        timestep_pub->publish(msg);
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "resetting timer");
        timestep = 0;
        response->success = true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NUSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}