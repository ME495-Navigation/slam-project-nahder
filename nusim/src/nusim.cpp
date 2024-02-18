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
// #include "sensor_msgs/msg/laser_scan"
#include "nav_msgs/msg/path.hpp"
#include <random>
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

    path_pub = create_publisher<nav_msgs::msg::Path>(
      "red/path", 10);

    fake_sensor_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10);

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
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 2.0);
    declare_parameter("collision_radius", 0.11);

    rate = get_parameter("rate").as_double();
    x0 = get_parameter("x0").as_double();
    y0 = get_parameter("y0").as_double();
    theta0 = get_parameter("theta0").as_double();
    arena_x_length = get_parameter("arena_x_length").as_double();
    arena_y_length = get_parameter("arena_y_length").as_double();
    obstacles_x = get_parameter("obstacles.x").as_double_array();
    obstacles_y = get_parameter("obstacles.y").as_double_array();
    obstacle_radius = get_parameter("obstacles.r").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    input_noise = get_parameter("input_noise").as_double();
    slip_fraction = get_parameter("slip_fraction").as_double();
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    max_range = get_parameter("max_range").as_double();
    collision_radius = get_parameter("collision_radius").as_double();

    red_x = x0, red_y = y0, red_theta = theta0;

    timer = create_wall_timer(1s / rate, std::bind(&NUSim::timer_callback, this));
    sensor_timer = create_wall_timer(0.2s, std::bind(&NUSim::sensor_timer_callback, this));

    auto walls_msg{create_walls(arena_x_length, arena_y_length)};
    walls_pub->publish(walls_msg);

    auto obstacles_msg{create_obstacles(obstacles_x, obstacles_y, obstacle_radius)};
    obstacles_pub->publish(obstacles_msg);
    dt = 1.0 / rate;

    noise_gaussian = std::normal_distribution<>(0.0, std::sqrt(input_noise));
    slip_gaussian = std::uniform_real_distribution<>{-1.0 * slip_fraction, slip_fraction};
    sensor_gaussian = std::normal_distribution<>(0.0, std::sqrt(basic_sensor_variance));
  }

private:
  rclcpp::TimerBase::SharedPtr timer, sensor_timer;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_srv;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr red_wheel_cmd_sub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub;

  turtlelib::DiffDrive red_diff_drive;
  nuturtlebot_msgs::msg::SensorData sensor_data;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  nav_msgs::msg::Path path_msg;
  std::normal_distribution<> noise_gaussian, sensor_gaussian;
  std::uniform_real_distribution<> slip_gaussian;

  double left_encoder_pos{0.0}, right_encoder_pos{0.0};
  double new_left_rads{0.0}, new_right_rads{0.0};
  turtlelib::wheelVel wheel_config{0.0, 0.0};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  geometry_msgs::msg::TransformStamped xform_stamped;
  tf2::Quaternion q;

  double x0, y0, theta0, red_x, red_y, red_theta, arena_x_length, arena_y_length, obstacle_radius,
    rate,
    motor_cmd_per_rad_sec, encoder_ticks_per_rad, dt, input_noise, slip_fraction,
    basic_sensor_variance, max_range, collision_radius;

  turtlelib::wheelVel red_wheel_vel{0.0, 0.0};

  std::vector<double> obstacles_x, obstacles_y;

  uint64_t timestep;

  std::mt19937 & get_random()
  {
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
  }

  /// @brief Main simulator loop. Publishes encoder values and updates the robot pose in TF
  void timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    timestep++;
    msg.data = timestep;
    timestep_pub->publish(msg);
    publish_encoders();
    update_robot_pose();
    collision_handling();
    update_xform();
  }

  void sensor_timer_callback()
  {
    publish_fake_sensor();

  }

  /// @brief Publishes MarkerArray onto the fake_sensor topic with measured obstacle positions (wrt robot)
  void publish_fake_sensor()
  {
    visualization_msgs::msg::MarkerArray perceived_obstacles;
    turtlelib::Transform2D T_world_to_robot{{red_x, red_y}, red_theta};
    auto T_robot_to_world = T_world_to_robot.inv();
    const auto obstacle_height{0.25};

    for (uint64_t i = 0; i < obstacles_x.size(); i++) {
      // apply noise and transform obstacle position from world to robot frame
      auto obstacle_pos = T_robot_to_world(
        turtlelib::Point2D
        {obstacles_x.at(i) + sensor_gaussian(get_random()),
          obstacles_y.at(i) + sensor_gaussian(get_random())}
      );

      auto dist = std::sqrt(std::pow(obstacle_pos.x, 2) + std::pow(obstacle_pos.y, 2));

      visualization_msgs::msg::Marker obs_marker;
      obs_marker.header.frame_id = "red/base_footprint";
      obs_marker.header.stamp = get_clock()->now();
      obs_marker.id = i;
      obs_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      obs_marker.scale.x = obstacle_radius;
      obs_marker.scale.y = obstacle_radius;
      obs_marker.scale.z = obstacle_height;
      obs_marker.pose.position.x = obstacle_pos.x;
      obs_marker.pose.position.y = obstacle_pos.y;
      obs_marker.pose.position.z = obstacle_height / 2;
      obs_marker.color.r = 1.0;
      obs_marker.color.g = 1.0;
      obs_marker.color.b = 0.0;
      obs_marker.color.a = 1.0;

      if (dist <= max_range) {
        obs_marker.action = visualization_msgs::msg::Marker::ADD;
      } else {
        obs_marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      perceived_obstacles.markers.push_back(obs_marker);
    }
    fake_sensor_pub->publish(perceived_obstacles);
  }

  /// @brief Handles obstacle collision, stopping the red robot and forcing odom wheel slippage
  void collision_handling()
  {
    double centers_dist{0.0}, intersect_amt{0.0};
    for (uint64_t i = 0; i < obstacles_x.size(); i++) {
      centers_dist = std::sqrt(
        std::pow(red_x - obstacles_x.at(i), 2) +
        std::pow(red_y - obstacles_y.at(i), 2)
      );
      intersect_amt = (collision_radius + obstacle_radius) - centers_dist;

      // collision happening
      if (intersect_amt > 0.0) {
        // vector between the robot and the obstacle
        auto v{
          turtlelib::Vector2D{red_x, red_y} -
          turtlelib::Vector2D{obstacles_x.at(i), obstacles_y.at(i)}
        };

        auto v_u = normalize_vector(v);
        // move back by how much we are intersecting along this vector
        auto new_pos = turtlelib::Vector2D{red_x, red_y} + (intersect_amt * v_u);

        red_diff_drive.set_config({new_pos.x, new_pos.y, red_theta});
      }
    }
  }

  /// @brief Callback for wheel commands (in MCU, [-265, 265])
  /// @param wheel_cmd_msg The wheel commands message
  /// @details Converts wheel commands from MCU to rad/s
  void wheel_cmd_cb(const nuturtlebot_msgs::msg::WheelCommands & wheel_cmd_msg)
  {
    auto noise = noise_gaussian(get_random());
    //calculate wheel commands in rad/s
    red_wheel_vel.left_wheel_vel = wheel_cmd_msg.left_velocity * motor_cmd_per_rad_sec;     // UNITS: RAD/S
    red_wheel_vel.right_wheel_vel = wheel_cmd_msg.right_velocity * motor_cmd_per_rad_sec;

    //add zero-mean Gaussian noise with variance input_noise when red_wheel_vel != 0 (confident in stopping)
    if (red_wheel_vel.left_wheel_vel != 0.0) {
      red_wheel_vel.left_wheel_vel += noise;
    }
    if (red_wheel_vel.right_wheel_vel != 0.0) {
      red_wheel_vel.right_wheel_vel += noise;
    }

  }
  /// @brief Publishes the current wheel encoder positions
  void publish_encoders()
  {
    auto slip = slip_gaussian(get_random());

    new_left_rads = red_wheel_vel.left_wheel_vel * (1.0 + slip) * dt;     // WHEEL POS UNITS: RADIANS
    new_right_rads = red_wheel_vel.right_wheel_vel * (1.0 + slip) * dt;     // WHEEL POS UNITS: RADIANS

    left_encoder_pos += new_left_rads * encoder_ticks_per_rad;     // UNITS: TICKS
    right_encoder_pos += new_right_rads * encoder_ticks_per_rad;     // UNITS: TICKS'

    sensor_data.left_encoder = left_encoder_pos;
    sensor_data.right_encoder = right_encoder_pos;
    sensor_data_pub->publish(sensor_data);     // PUBLISH CURRENT WHEEL ENCODER POSITIONS IN TICKS
  }

  /// @brief Updates the robot pose based on the wheel positions
  void update_robot_pose()
  {
    wheel_config = turtlelib::wheelVel{new_right_rads, new_left_rads};
    red_diff_drive.forwardKinematics(wheel_config);
    red_x = red_diff_drive.get_config().x;
    red_y = red_diff_drive.get_config().y;
    red_theta = red_diff_drive.get_config().theta;
  }

  /// @brief Publishes the current robot pose to TF and the path
  void update_xform()
  {
    xform_stamped.header.stamp = get_clock()->now();
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

    path_msg.header.stamp = get_clock()->now();
    path_msg.header.frame_id = "nusim/world";
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = red_x;
    pose_stamped.pose.position.y = red_y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();
    path_msg.poses.push_back(pose_stamped);
    path_pub->publish(path_msg);
  }

  /// @brief Restarts the simulation by resetting the robot pose and the timestep
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO_STREAM(get_logger(), "resetting timestep and pose");

    red_x = x0, red_y = y0, red_theta = theta0, timestep = 0;
    left_encoder_pos = 0, right_encoder_pos = 0;
    red_diff_drive.set_config({red_x, red_y, red_theta});
    response->success = true;
  }

  /// @brief Teleports the robot in simulation to a new pose
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "teleporting to " << request->x << ", " << request->y << ", " << request->theta);

    red_x = request->x, red_y = request->y, red_theta = request->theta;
    red_diff_drive.set_config({red_x, red_y, red_theta});
    response->success = true;
  }

  /// @brief Create walls for visualization
  /// @param x_length The length of the arena in the x direction
  /// @param y_length The length of the arena in the y direction
  /// @return visualization_msgs::msg::MarkerArray
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
        wall.header.stamp = get_clock()->now();
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

  /// @brief Create obstacles for visualization
  /// @param x x positions of obstacles
  /// @param y y positions of obstacles
  /// @param r obstacle radii
  /// @return visualization_msgs::msg::MarkerArray
  visualization_msgs::msg::MarkerArray create_obstacles(
    std::vector<double> x, std::vector<double> y, double r)
  {

    if (x.size() != y.size()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
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
        obstacle.header.stamp = get_clock()->now();
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
