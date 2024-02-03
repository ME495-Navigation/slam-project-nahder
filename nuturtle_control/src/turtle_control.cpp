#include <cstdio>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl() : Node("turtle_control")
  {
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_max", 0.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("collision_radius", 0.0);

    // Assign parameter values to member variables
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius = get_parameter("collision_radius").as_double();

    check_params();
  }

private:
  double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, 
  encoder_ticks_per_rad, collision_radius;


  void check_params()
  {
    if (wheel_radius == 0.0 || track_width == 0.0 || motor_cmd_max == 0.0 || 
        motor_cmd_per_rad_sec == 0.0 || encoder_ticks_per_rad == 0.0 || collision_radius == 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Missing parameter(s)");
      // rclcpp::shutdown(); uncomment after params loaded into param server
    }
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}