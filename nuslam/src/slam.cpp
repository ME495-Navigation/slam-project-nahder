#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/ekf.hpp"

using namespace std::chrono_literals;
class Slam : public rclcpp::Node
{

public
  :
  Slam()
  : Node("nuslam")
  {
    declare_parameter("slam_body_id", "");         // base_footprint
    declare_parameter("slam_odom_id", "");     // o

    declare_parameter("wheel_left", "");      // wheel_left_joint
    declare_parameter("wheel_right", "");     // wheel_right_joint

    declare_parameter("wheel_radius", 0.0);
    declare_parameter("track_width", 0.0);

    slam_body_id = get_parameter("slam_body_id").as_string(); // green /base_footprint
    slam_odom_id = get_parameter("slam_odom_id").as_string(); // green /odom

    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();

    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();

    check_params();

    turtleBot = turtlelib::DiffDrive(wheel_radius, track_width);
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("~/odom", 10);

    joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Slam::joint_state_callback, this, std::placeholders::_1));

    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    initial_pose_srv = create_service<nuturtle_control::srv::InitialPose>(
      "~/initial_pose", std::bind(
        &Slam::initial_pose_callback,
        this, std::placeholders::_1, std::placeholders::_2));

    fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));

    obstacle_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacle_map",
      10);

    prev_js_msg.position = {0.0, 0.0};
  }

private:
  std::string slam_body_id, wheel_left, wheel_right, slam_odom_id;
  double wheel_radius, track_width;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_pub;

  rclcpp::TimerBase::SharedPtr timer;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, tf_map_broadcaster;
  turtlelib::DiffDrive turtleBot;

  sensor_msgs::msg::JointState prev_js_msg;
  turtlelib::EKF ekf;
  turtlelib::Transform2D T_odom_robot, T_map_odom, T_map_robot;

  void check_params()
  {
    if (slam_body_id.empty() || wheel_left.empty() || wheel_right.empty() ||
      wheel_radius == 0.0 || track_width == 0.0)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Missing SLAM parameter(s). Exiting...");
      throw std::runtime_error("Missing SLAM parameter(s). Exiting...");
    }
  }

// turtle_control node converts encoder ticks to radians and publishes joint state messages {left_wheel_joint, right_wheel_joint}
//  every time a joint state message is received, update internal odometry state and publish odometry message
//  odometry: relative position and orientation of the robot from where it started
  void joint_state_callback(const sensor_msgs::msg::JointState & js_msg)
  {
    // joint state msg has the position and velocity for each wheel
    // take the joint state msg, turn it into a wheelVel object, and pass it to forwardKinematics
    turtlelib::wheelVel new_wheel_config{
      js_msg.position.at(1) - prev_js_msg.position.at(1),       // in radians
      js_msg.position.at(0) - prev_js_msg.position.at(0)};

    // use forward kinematics to update robot configuration given the new wheel configuration
    turtleBot.forwardKinematics(new_wheel_config);
    auto config = turtleBot.get_config();
    auto body_twist = turtleBot.computeBodyTwist(new_wheel_config);

    // publish the new robot configuration as an odometry message
    nav_msgs::msg::Odometry odom_msg;
    // format an odometry message with the new robot configuration and publish it
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.header.frame_id = slam_odom_id;
    odom_msg.child_frame_id = slam_body_id;
    odom_msg.pose.pose.position.x = config.x;
    odom_msg.pose.pose.position.y = config.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.twist.twist.linear.x = body_twist.x;
    odom_msg.twist.twist.linear.y = body_twist.y;
    odom_msg.twist.twist.angular.z = body_twist.omega;

    tf2::Quaternion q, q_map;
    q.setRPY(0, 0, config.theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    odom_pub->publish(odom_msg);

    // publish the new robot configuration as a transform
    geometry_msgs::msg::TransformStamped odom_xform;
    odom_xform.header.stamp = get_clock()->now();
    odom_xform.header.frame_id = slam_odom_id;
    odom_xform.child_frame_id = slam_body_id;
    odom_xform.transform.translation.x = config.x;
    odom_xform.transform.translation.y = config.y;
    odom_xform.transform.translation.z = 0.0;
    odom_xform.transform.rotation.x = q.x();
    odom_xform.transform.rotation.y = q.y();
    odom_xform.transform.rotation.z = q.z();
    odom_xform.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(odom_xform);
    prev_js_msg = js_msg;

    T_odom_robot = turtlelib::Transform2D{
      turtlelib::Vector2D{config.x, config.y}, config.theta};

    // T_map_robot found from EKF slam vector
    T_map_odom = T_map_robot * T_odom_robot.inv();

    geometry_msgs::msg::TransformStamped map_odom_xform;
    map_odom_xform.header.stamp = get_clock()->now();
    map_odom_xform.header.frame_id = "map";
    map_odom_xform.child_frame_id = slam_odom_id;
    map_odom_xform.transform.translation.x = T_map_odom.translation().x;
    map_odom_xform.transform.translation.y = T_map_odom.translation().y;
    map_odom_xform.transform.translation.z = 0.0;
    q_map.setRPY(0, 0, T_map_odom.rotation());
    map_odom_xform.transform.rotation.x = q_map.x();
    map_odom_xform.transform.rotation.y = q_map.y();
    map_odom_xform.transform.rotation.z = q_map.z();
    map_odom_xform.transform.rotation.w = q_map.w();
    tf_map_broadcaster->sendTransform(map_odom_xform);
  }

// service to set the initial pose of the robot. odometry should be updated to reflect the new pose
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response> response)
  {
    turtlelib::robotConfig config{request->x, request->y, request->theta};
    turtleBot.set_config(config);

    RCLCPP_INFO_STREAM(
      get_logger(), "Setting initial pose to x: " << request->x << " y: "
                                                  << request->y << " theta: " << request->theta);
    response->success = true;
  }

  // nusim node publishes fake sensor data as a marker array
  // the marker array is obstacles within the lidar range of the robot
  // take the perceived obstacles and use them for ekf.update() to update the state vector {robot_pos,map}
  // update T_map_robot for the xform broadcaster
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr lidar_data_msg)
  {
    auto config = turtleBot.get_config();

    //ekf predict takes in the current robot twist (using its configuration,t=1)
    ekf.predict(turtlelib::Twist2D{config.x, config.y, config.theta});

    //ekf update takes in the perceived obstacles and index of the perceived obstacles
    for (size_t i = 0; i < lidar_data_msg->markers.size(); i++) {
      auto marker = lidar_data_msg->markers[i];
      if (marker.action == visualization_msgs::msg::Marker::ADD) {
        ekf.update(marker.pose.position.x, marker.pose.position.y, i);
      }
    }
    auto state = ekf.get_state();
    auto robot_pos = turtlelib::Vector2D{state(1), state(2)};
    auto robot_theta = state(0);
    T_map_robot = turtlelib::Transform2D{robot_pos, robot_theta};

    //publish the SLAM estimated obstacles as a marker array
    visualization_msgs::msg::MarkerArray obstacles_arr;
    for (size_t i = 0; i < lidar_data_msg->markers.size(); i++) {
      visualization_msgs::msg::Marker obstacle;
      obstacle.header.frame_id = "nusim/world";
      obstacle.header.stamp = get_clock()->now();
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.id = lidar_data_msg->markers.at(i).id + 999;
      obstacle.pose.position.x = state(3 + 2 * i);
      obstacle.pose.position.y = state(4 + 2 * i);
      obstacle.pose.position.z = lidar_data_msg->markers.at(i).pose.position.z;
      obstacle.scale.x = lidar_data_msg->markers.at(i).scale.x;
      obstacle.scale.y = lidar_data_msg->markers.at(i).scale.y;
      obstacle.scale.z = lidar_data_msg->markers.at(i).scale.z;
      obstacle.color.g = 1.0;
      obstacle.color.a = 1.0;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacles_arr.markers.push_back(obstacle);
    }
    obstacle_pub->publish(obstacles_arr);

  }


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
