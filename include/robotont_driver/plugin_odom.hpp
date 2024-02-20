// include/robotont_driver/plugin_odom.hpp:

#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#ifndef ODOM_HPP
#define ODOM_HPP

namespace robotont
{
// PluginOdom class
class PluginOdom
{
public:
  // Constructor that takes a shared pointer to a node
  PluginOdom(rclcpp::Node::SharedPtr node_);

  // Destructor
  ~PluginOdom();

  // Function to reset the odom message values
  void reset();

  // Function to receive the packet and transform it into an odom message
  void packetReceived(const std::vector<std::string>& packet);

  // Function to publish the odometry and the TF messages
  void publish();

  // Function to update the odometry
  void update(float pos_x, float pos_y, float ori_z, float lin_vel_x, float lin_vel_y, float ang_vel_z);

  // Function to set the parent frame name for odom and its tf
  void setFrameId(const std::string& frame_id);

  // Function to set the child frame name for odom and its tf
  void setChildFrameId(const std::string& child_frame_id);

private:
  // Shared pointer to the node
  rclcpp::Node::SharedPtr node_;
  // Unique pointer to the odometry message
  nav_msgs::msg::Odometry::UniquePtr odom_msg_;

  // Unique pointer to the transform message
  geometry_msgs::msg::TransformStamped::UniquePtr odom_transform_;

  // Shared pointer to the odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Shared pointer to the broadcaster class for publishing transform messages
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odom_broadcaster_;

  // Weak pointer to the driver node
  rclcpp::Node::WeakPtr weak_node_;
};
// Typedef for a shared pointer to a PluginOdom object
typedef std::shared_ptr<PluginOdom> OdomPtr;
} // namespace robotont

#endif