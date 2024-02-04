#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

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

    turtleBot = turtlelib::DiffDrive(wheel_radius, track_width);  

    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    check_params();
  }

private:
  double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, 
  encoder_ticks_per_rad, collision_radius;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;

  nuturtlebot_msgs::msg::WheelCommands wheelMsg;
  turtlelib::DiffDrive turtleBot;

  void check_params()
  {
    if (wheel_radius == 0.0 || track_width == 0.0 || motor_cmd_max == 0.0 || 
        motor_cmd_per_rad_sec == 0.0 || encoder_ticks_per_rad == 0.0 || collision_radius == 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Missing parameter(s)");
      // rclcpp::shutdown(); uncomment after params loaded into param server
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    //TODO
    // takes a cmd_vel and converts it to wheel commands of type nuturtlebot_msgs::msg::WheelCommands
    // publishes it to /wheel_cmd for the turtlebot to execute

    //twist consists of omegaDot, xDot, yDot
    
    //inverse kinematics returns the wheel velocities given a desired robot twist 

    
  }



};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}