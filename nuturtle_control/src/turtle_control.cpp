#include <cstdio>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);
    declare_parameter("motor_cmd_max", 0.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("collision_radius", 0.0);

    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    collision_radius = get_parameter("collision_radius").as_double();

    check_params();

    turtleBot = turtlelib::DiffDrive(wheel_radius, track_width);

    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    sensor_data_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    wheel_cmd_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("/wheel_cmd", 10);

    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

private:
  double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec,
    encoder_ticks_per_rad, collision_radius;

  double prev_sensor_time{-1.0};
  double left_prev{0.0}, right_prev{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub;

  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;

  turtlelib::DiffDrive turtleBot;

  void check_params()
  {
    if (wheel_radius == 0.0 || track_width == 0.0 || motor_cmd_max == 0.0 ||
      motor_cmd_per_rad_sec == 0.0 || encoder_ticks_per_rad == 0.0 || collision_radius == 0.0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Missing turtle_control parameter(s). Exiting...");
      throw std::runtime_error("Missing odometry parameter(s). Exiting...");
    }
  }

  /// @brief takes in a body twist and publishes wheel vel commands in MCU to /wheel_cmd
  /// @param geometry_msg type twist
  void cmd_vel_callback(const geometry_msgs::msg::Twist & twist)
  {
    // convert geometry msg twist to Twist2D twist
    turtlelib::Twist2D tw{twist.angular.z, twist.linear.x, twist.linear.y};

    // convert Twist2D twist to wheelVel u
    turtlelib::wheelVel u = turtleBot.inverseKinematics(tw);
    u.left_wheel_vel /= motor_cmd_per_rad_sec;
    u.right_wheel_vel /= motor_cmd_per_rad_sec;

    // clamp wheel velocities
    u.left_wheel_vel = std::clamp(u.left_wheel_vel, -motor_cmd_max, motor_cmd_max);
    u.right_wheel_vel = std::clamp(u.right_wheel_vel, -motor_cmd_max, motor_cmd_max);

    // convert wheelVel u to nuturtlebot_msgs::msg::WheelCommands wheelMsg
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd_msg;
    wheel_cmd_msg.left_velocity = u.left_wheel_vel;
    wheel_cmd_msg.right_velocity = u.right_wheel_vel;

    // publish wheelMsg
    wheel_cmd_pub->publish(wheel_cmd_msg); //IN MCU
  }

  /// @brief takes in wheel encoder readings and publishes joint states to /joint_states
  /// @param sensor_data
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    auto cur_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9;
    double dt{0.0}, left_velocity{0.0}, right_velocity{0.0};

    double left_position = static_cast<double>(msg.left_encoder) / encoder_ticks_per_rad;
    double right_position = static_cast<double>(msg.right_encoder) / encoder_ticks_per_rad;

    // check if not the first message
    if (prev_sensor_time > 0.0) {
      dt = cur_time - prev_sensor_time;
      left_velocity = (left_position - left_prev) / dt;
      right_velocity = (right_position - right_prev) / dt;
    }
    sensor_msgs::msg::JointState joint_states_msg;
    joint_states_msg.header.stamp = get_clock()->now();
    joint_states_msg.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_states_msg.position = {left_position, right_position}; // in rad
    joint_states_msg.velocity = {left_velocity, right_velocity}; // in rad/s
    joint_states_pub->publish(joint_states_msg);

    // update the prevs for next iteration
    left_prev = left_position;
    right_prev = right_position;
    prev_sensor_time = cur_time;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
