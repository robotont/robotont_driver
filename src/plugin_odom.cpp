#include "robotont_driver/plugin_odom.hpp"

using namespace std::chrono_literals;

namespace robotont
{
// Constructor for the PluginOdom class
PluginOdom::PluginOdom(rclcpp::Node::SharedPtr node_) : node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont odometry is starting...");
  // Create messages
  odom_msg_ = std::make_unique<nav_msgs::msg::Odometry>();
  odom_transform_ = std::make_unique<geometry_msgs::msg::TransformStamped>();

  // Set default frame names for odom and robot's base.
  setFrameId("odom");
  setChildFrameId("base_footprint");

  // Initialize messages
  reset();

  // Initialize odom publisher
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 2);
}

// Destructor for the PluginOdom class
PluginOdom::~PluginOdom()
{
}

// Create Odom package from the data received from serial port
void PluginOdom::packetReceived(const std::vector<std::string>& packet)
{
  if (packet.size() != 7 || packet[0] != "ODOM")
  {
    return;
  }

  float pos_x, pos_y, ori_z, lin_vel_x, lin_vel_y, ang_vel_z;

  try
  {
    pos_x = std::stof(packet[1]);
    pos_y = std::stof(packet[2]);
    ori_z = std::stof(packet[3]);
    lin_vel_x = std::stof(packet[4]);
    lin_vel_y = std::stof(packet[5]);
    ang_vel_z = std::stof(packet[6]);
  }
  catch (std::exception e)
  {
    RCLCPP_ERROR(node_->get_logger(), "ODOM error reading packet");
  }

  odom_msg_->header.stamp = node_->now();
  odom_msg_->pose.pose.position.x = pos_x;
  odom_msg_->pose.pose.position.y = pos_y;
  odom_msg_->pose.pose.position.z = ori_z;
  odom_msg_->pose.pose.orientation.x = lin_vel_x;
  odom_msg_->pose.pose.orientation.y = lin_vel_y;
  odom_msg_->pose.pose.orientation.z = ang_vel_z;
  odom_msg_->pose.pose.orientation.w = 1;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ori_z);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);
  odom_transform_->header.stamp = node_->now();
  odom_transform_->transform.translation.x = pos_x;
  odom_transform_->transform.translation.y = pos_y;
  odom_transform_->transform.translation.z = 0;
  odom_transform_->transform.rotation = quat_msg;

  publish();
}

void PluginOdom::setFrameId(const std::string& frame_id)
{
  odom_msg_->header.frame_id = frame_id;
  odom_transform_->header.frame_id = frame_id;
}

void PluginOdom::setChildFrameId(const std::string& child_frame_id)
{
  odom_msg_->child_frame_id = child_frame_id;
  odom_transform_->child_frame_id = child_frame_id;
}

// Publish the odom message to odom topic
void PluginOdom::publish()
{
  if (odom_pub_)
  {
    (odom_pub_->publish)(*odom_msg_);
  }
  if (odom_broadcaster_)
  {
    (odom_broadcaster_->sendTransform)(*odom_transform_);
  }
}

// Clear odom message
void PluginOdom::reset()
{
  odom_msg_->header.stamp = node_->now();
  odom_msg_->pose.pose.position.x = 0;
  odom_msg_->pose.pose.position.y = 0;
  odom_msg_->pose.pose.position.z = 0;
  odom_msg_->pose.pose.orientation.x = 0;
  odom_msg_->pose.pose.orientation.y = 0;
  odom_msg_->pose.pose.orientation.z = 0;
  odom_msg_->pose.pose.orientation.w = 1;

  odom_transform_->header.stamp = node_->now();
  odom_transform_->transform.translation.x = 0;
  odom_transform_->transform.translation.y = 0;
  odom_transform_->transform.translation.z = 0;
  odom_transform_->transform.rotation.x = 0;
  odom_transform_->transform.rotation.y = 0;
  odom_transform_->transform.rotation.z = 0;
  odom_transform_->transform.rotation.w = 0;
}

} // namespace robotont