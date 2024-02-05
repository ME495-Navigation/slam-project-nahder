#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{

public:
    Odometry() : Node("odometry")
    {
        declare_parameter("body_id", "");     // base_footprint
        declare_parameter("wheel_left", "");  // wheel_left_joint
        declare_parameter("wheel_right", ""); // wheel_right_joint
        declare_parameter("odom_id", "odom"); // odom

        declare_parameter("wheel_radius", 0.0);
        declare_parameter("track_width", 0.0);

        body_id = get_parameter("body_id").as_string();
        wheel_left = get_parameter("wheel_left").as_string();
        wheel_right = get_parameter("wheel_right").as_string();
        odom_id = get_parameter("odom_id").as_string();

        wheel_radius = get_parameter("wheel_radius").as_double();
        track_width = get_parameter("track_width").as_double();

        check_params();

        turtleBot = turtlelib::DiffDrive(wheel_radius, track_width);
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_state_sub = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this); 

        prev_js_msg.position = {0.0, 0.0};
    }

private:
    std::string body_id, wheel_left, wheel_right, odom_id;
    double wheel_radius, track_width;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    turtlelib::DiffDrive turtleBot;

    sensor_msgs::msg::JointState prev_js_msg;

    void check_params()
    {
        if (body_id.empty() || wheel_left.empty() || wheel_right.empty() ||
            wheel_radius == 0.0 || track_width == 0.0)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Missing odometry parameter(s). Exiting...");
            rclcpp::shutdown();
        }
    }

    //turtle_control node converts encoder ticks to radians and publishes joint state messages {left_wheel_joint, right_wheel_joint}
    // every time a joint state message is received, update internal odometry state and publish odometry message
    // odometry: relative position and orientation of the robot from where it started
    void joint_state_callback(const sensor_msgs::msg::JointState &js_msg)
    {
        // joint state msg has the position and velocity for each wheel
        turtlelib::wheelVel new_wheel_config{
            js_msg.position[0] - prev_js_msg.position[0], // in radians
            js_msg.position[1] - prev_js_msg.position[1]};

        // use forward kinematics to update robot configuration given the new wheel configuration
        // take the joint state msg, turn it into a wheelVel object, and pass it to forwardKinematics
        turtleBot.forwardKinematics(new_wheel_config);
        turtlelib::robotConfig config = turtleBot.get_config();
        turtlelib::Twist2D body_twist = turtleBot.computeBodyTwist(new_wheel_config);

        // publish the new robot configuration as an odometry message
        nav_msgs::msg::Odometry odom_msg;

        // format an odometry message with the new robot configuration and publish it
        odom_msg.header.stamp = js_msg.header.stamp;
        odom_msg.header.frame_id = odom_id;
        odom_msg.child_frame_id = body_id;
        odom_msg.pose.pose.position.x = config.x;
        odom_msg.pose.pose.position.y = config.y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.twist.twist.linear.x = body_twist.x;
        odom_msg.twist.twist.linear.y = body_twist.y;
        odom_msg.twist.twist.angular.z = body_twist.omega;
        tf2::Quaternion q;
        q.setRPY(0, 0, config.theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_pub->publish(odom_msg);

        // publish the new robot configuration as a transform
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = js_msg.header.stamp;
        odom_tf.header.frame_id = odom_id;
        odom_tf.child_frame_id = body_id;
        odom_tf.transform.translation.x = config.x;
        odom_tf.transform.translation.y = config.y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(odom_tf);

        prev_js_msg = js_msg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odometry>());
    rclcpp::shutdown();
    return 0;
}
