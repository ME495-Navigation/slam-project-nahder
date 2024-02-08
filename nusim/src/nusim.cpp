#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/diff_drive.hpp"

#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;

class NUSim : public rclcpp::Node
{
public:
  NUSim()
  : Node("nusim")
  {
    timestep = 0;

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timestep_pub = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    reset_srv = create_service<std_srvs::srv::Trigger>(
      "~/reset",
      std::bind(
        &NUSim::reset_callback,
        this, std::placeholders::_1,
        std::placeholders::_2));

    teleport_srv = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(
        &NUSim::teleport_callback,
        this, std::placeholders::_1,
        std::placeholders::_2));

    walls_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      rclcpp::QoS(1).transient_local());

    obstacles_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      rclcpp::QoS(1).transient_local());

    red_wheel_cmd_sub = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&NUSim::wheel_cmd_cb, this, std::placeholders::_1));

    sensor_data_pub = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data", 10);

    declare_parameter("rate", 200.0);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 10.0);
    declare_parameter("arena_y_length", 10.0);
    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    declare_parameter("encoder_ticks_per_rad", 0.0);
    declare_parameter("obstacles.x", std::vector<double>{1.0, 2.0, 3.0});
    declare_parameter("obstacles.y", std::vector<double>{1.0, 2.0, 3.0});
    declare_parameter("obstacles.r", 0.5);

    rate = get_parameter("rate").as_double();
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();
    obstacles_x = get_parameter("obstacles.x").as_double_array();
    obstacles_y = get_parameter("obstacles.y").as_double_array();
    obstacle_r = get_parameter("obstacles.r").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

    red_x = x0, red_y = y0, red_theta = theta0;

    timer = create_wall_timer(1s / rate, std::bind(&NUSim::timer_callback, this));

    auto walls_msg{create_walls(arena_x_length, arena_y_length)};
    walls_pub->publish(walls_msg);

    auto obstacles_msg{create_obstacles(obstacles_x, obstacles_y, obstacle_r)};
    obstacles_pub->publish(obstacles_msg);
    dt = 1.0 / rate;
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  turtlelib::DiffDrive red_diff_drive;
  nuturtlebot_msgs::msg::SensorData sensor_data;

  double left_encoder_pos{0.0}, right_encoder_pos{0.0};
  double new_left_rads{0.0}, new_right_rads{0.0};
  turtlelib::wheelVel wheel_config{0.0, 0.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  geometry_msgs::msg::TransformStamped xform_stamped;
  tf2::Quaternion q;

  double x0, y0, theta0, red_x, red_y, red_theta, arena_x_length,
    arena_y_length, obstacle_r, rate, motor_cmd_per_rad_sec, encoder_ticks_per_rad, dt;

  turtlelib::wheelVel red_wheel_vel{0.0, 0.0};

  std::vector<double> obstacles_x, obstacles_y;

  uint64_t timestep;

  void timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    timestep++;
    msg.data = timestep;
    timestep_pub->publish(msg);
    publish_encoders();
    update_robot_pose();
    update_xform();
  }

  // STORE WHEEL COMMANDS IN RAD/SEC (converted from MCU)
  void wheel_cmd_cb(const nuturtlebot_msgs::msg::WheelCommands & wheel_cmd_msg)
  {
    red_wheel_vel.left_wheel_vel = wheel_cmd_msg.left_velocity * motor_cmd_per_rad_sec; // UNITS: radians/sec
    red_wheel_vel.right_wheel_vel = wheel_cmd_msg.right_velocity * motor_cmd_per_rad_sec;
  }

  // PUBLISH CURRENT WHEEL ENCODER POSITIONS IN TICKS
  // MOVE THE ROBOT ACCORDINGLY
  void publish_encoders()
  {
    new_left_rads = red_wheel_vel.left_wheel_vel * dt;           // WHEEL POS UNITS: RADIANS
    new_right_rads = red_wheel_vel.right_wheel_vel * dt;         // WHEEL POS UNITS: RADIANS
    left_encoder_pos += new_left_rads * encoder_ticks_per_rad;   // UNITS: TICKS
    right_encoder_pos += new_right_rads * encoder_ticks_per_rad; // UNITS: TICKS
    sensor_data.left_encoder = left_encoder_pos;
    sensor_data.right_encoder = right_encoder_pos;
    sensor_data_pub->publish(sensor_data); // PUBLISH CURRENT WHEEL ENCODER POSITIONS IN TICKS
  }

  void update_robot_pose()
  {
    wheel_config = turtlelib::wheelVel{new_right_rads, new_left_rads}; // UPDATE ROBOT POSE GIVEN WHEEL CONFIGURATION
    red_diff_drive.forwardKinematics(wheel_config);
    red_x = red_diff_drive.get_config().x;
    red_y = red_diff_drive.get_config().y;
    red_theta = red_diff_drive.get_config().theta;
  }

  void update_xform()
  {
    xform_stamped.header.stamp = this->get_clock()->now();
    xform_stamped.header.frame_id = "nusim/world";
    xform_stamped.child_frame_id = "red/base_footprint";
    xform_stamped.transform.translation.x = red_x;
    xform_stamped.transform.translation.y = red_y;
    xform_stamped.transform.translation.z = 0.0;
    q.setRPY(0, 0, red_theta);
    xform_stamped.transform.rotation.x = q.x();
    xform_stamped.transform.rotation.y = q.y();
    xform_stamped.transform.rotation.z = q.z();
    xform_stamped.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(xform_stamped);
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "resetting timestep and pose");

    red_x = x0, red_y = y0, red_theta = theta0, timestep = 0;
    response->success = true;
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "teleporting to " << request->x << ", " << request->y << ", " << request->theta);

    red_x = request->x, red_y = request->y, red_theta = request->theta;
    response->success = true;
  }

  visualization_msgs::msg::MarkerArray create_walls(double x_length, double y_length)
  {
    visualization_msgs::msg::MarkerArray walls;

    auto wall_height{0.25};
    auto wall_thickness{0.15};

    auto create_wall = [&](int id, double scale_x, double scale_y,
        double pos_x, double pos_y)
      {
        visualization_msgs::msg::Marker wall;
        wall.header.frame_id = "nusim/world";
        wall.header.stamp = this->get_clock()->now();
        wall.ns = "arena_walls";
        wall.id = id;
        wall.type = visualization_msgs::msg::Marker::CUBE;
        wall.action = visualization_msgs::msg::Marker::ADD;
        wall.scale.x = scale_x;
        wall.scale.y = scale_y;
        wall.scale.z = wall_height;
        wall.pose.position.x = pos_x;
        wall.pose.position.y = pos_y;
        wall.pose.position.z = wall_height / 2;
        wall.color.r = 1.0;
        wall.color.a = 1.0;
        return wall;
      };

    walls.markers.push_back(
      create_wall(
        0, x_length + 2 * wall_thickness, wall_thickness,
        0, -y_length / 2 - wall_thickness / 2));
    walls.markers.push_back(
      create_wall(
        1, x_length + 2 * wall_thickness, wall_thickness,
        0, y_length / 2 + wall_thickness / 2));
    walls.markers.push_back(
      create_wall(
        2, wall_thickness, y_length + 2 * wall_thickness,
        -x_length / 2 - wall_thickness / 2, 0));
    walls.markers.push_back(
      create_wall(
        3, wall_thickness, y_length + 2 * wall_thickness,
        x_length / 2 + wall_thickness / 2, 0));

    return walls;
  }

  visualization_msgs::msg::MarkerArray create_obstacles(
    std::vector<double> x, std::vector<double> y, double r)
  {

    if (x.size() != y.size()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "number of x and y coordinates must be the same");

      throw std::runtime_error("obstacle list size mismatch, exiting");
    }

    visualization_msgs::msg::MarkerArray obstacles;
    auto obstacle_height{0.25};

    auto create_obstacle = [&](int id, double scale_x, double scale_y,
        double pos_x, double pos_y)
      {
        visualization_msgs::msg::Marker obstacle;
        obstacle.header.frame_id = "nusim/world";
        obstacle.header.stamp = this->get_clock()->now();
        obstacle.ns = "obstacles";
        obstacle.id = id;
        obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
        obstacle.action = visualization_msgs::msg::Marker::ADD;
        obstacle.scale.x = scale_x;
        obstacle.scale.y = scale_y;
        obstacle.scale.z = obstacle_height;
        obstacle.pose.position.x = pos_x;
        obstacle.pose.position.y = pos_y;
        obstacle.pose.position.z = obstacle_height / 2;
        obstacle.color.r = 1.0;
        obstacle.color.a = 1.0;
        return obstacle;
      };

    for (uint64_t i = 0; i < x.size(); i++) {
      obstacles.markers.push_back(create_obstacle(i, r, r, x[i], y[i]));
    }

    return obstacles;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NUSim>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
