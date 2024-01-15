#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;

class NUSim : public rclcpp::Node
{

public:
    NUSim() : Node("nusim")
    { 
        timestep = 0;

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timestep_pub = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        reset_srv = create_service<std_srvs::srv::Trigger>("~/reset",
                                                           std::bind(&NUSim::reset_callback,
                                                                     this, std::placeholders::_1,
                                                                     std::placeholders::_2));

        teleport_srv = create_service<nusim::srv::Teleport>("~/teleport",
                                                            std::bind(&NUSim::teleport_callback,
                                                                      this, std::placeholders::_1,
                                                                      std::placeholders::_2));
        declare_parameter("rate", 200.0);
        declare_parameter("x0", 0.0);
        declare_parameter("y0", 0.0);
        declare_parameter("theta0", 0.0);

        rate = get_parameter("rate").as_double();
        x0 = get_parameter("x0").as_double();
        y0 = get_parameter("y0").as_double();
        theta0 = get_parameter("theta0").as_double();

        cur_x = x0, cur_y = y0, cur_theta = theta0;

        timer = create_wall_timer(1s / rate, std::bind(&NUSim::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped xform_stamped;
    tf2::Quaternion q;

    double x0, y0, theta0, cur_x, cur_y, cur_theta;
    double rate;
    uint64_t timestep;
 
    void timer_callback()
    {
        std_msgs::msg::UInt64 msg;
        timestep++;
        msg.data = timestep;
        timestep_pub->publish(msg);

        xform_stamped.header.stamp = this->get_clock()->now();
        xform_stamped.header.frame_id = "nusim/world";
        xform_stamped.child_frame_id = "red/base_footprint";
        xform_stamped.transform.translation.x = cur_x;
        xform_stamped.transform.translation.y = cur_y;
        xform_stamped.transform.translation.z = 0.0;
        q.setRPY(0, 0, cur_theta);
        xform_stamped.transform.rotation.x = q.x();
        xform_stamped.transform.rotation.y = q.y();
        xform_stamped.transform.rotation.z = q.z();
        xform_stamped.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(xform_stamped);
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "resetting timestep");
        cur_x = x0, cur_y = y0, cur_theta = theta0, timestep = 0;
        response->success = true;
    }

    void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
                           std::shared_ptr<nusim::srv::Teleport::Response> response)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "teleporting to " << request->x << ", " << request->y << ", " << request->theta);
            
        cur_x = request->x, cur_y = request->y, cur_theta = request->theta;
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