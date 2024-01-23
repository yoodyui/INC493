#include "vesc_ackermann/vesc_to_odom.hpp"
#include <cmath>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>


namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
    : Node("vesc_to_odom_node", options),
      odom_frame_("odom"),
      base_frame_("base_link"),
      publish_tf_(true), // Always publish TF
      x_(0.0),
      y_(0.0),
      initial_yaw_(0.0) // Initialize initial_yaw_ here
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  //BP subscribe to yaw from arduino
  yaw_sub_ = create_subscription<std_msgs::msg::Float64>("yaw", 10, std::bind(&VescToOdom::yawCallback, this, _1));

  //BP subscribe to current_speed from arduino
  speed_sub_ = create_subscription<std_msgs::msg::Float64>("xvel", 10, std::bind(&VescToOdom::speedCallback, this, _1));
        

  // Set the update rate to 10 Hz (0.1 seconds)
  update_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescToOdom::updateCallback, this));
}

// BP Define callback functions for each message
void VescToOdom::yawCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  if (initial_yaw_ == 0.0) {
    initial_yaw_ = msg->data;
  }
  yaw_ = msg->data - initial_yaw_; // Calculate relative yaw
}

//BP
void VescToOdom::speedCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  current_speed = msg->data;
}

void VescToOdom::updateCallback()
{
  // This function is called every 0.1 seconds, update your odometry here
  double current_steering_angle(0.0), current_angular_velocity(0.0);


    
    
  // propigate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  double dt = 0.1; //set sampling time to 0.1 sec
  x_ += x_dot * dt;
  y_ += y_dot * dt;
    
    
  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = now();
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom.pose.pose.orientation.w = cos(yaw_ / 2.0);


  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = current_speed;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = current_angular_velocity;

  
  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
