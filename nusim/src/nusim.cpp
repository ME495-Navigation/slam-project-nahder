#include "rclcpp/rclcpp.hpp"

// https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node
using namespace std::chrono_literals;

class NUSim : public rclcpp::Node
{

public:
    NUSim(): Node("nusim") { //constructor

    declare_parameter("nusim.nu", 0.0);


    RCLCPP_INFO(this->get_logger(), "hello world %d", 4);
    RCLCPP_INFO_STREAM(this->get_logger(), "helo world " << 4);

    }



private:


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NUSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}