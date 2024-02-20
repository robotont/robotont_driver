#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "robotont_driver/plugin_base.hpp"

#ifndef MOTORS_HPP
#define MOTORS_HPP

using namespace std::chrono_literals;

namespace robotont
{
// PluginMotors class
class PluginMotors
{
public:
  // Constructor that takes a HardwarePtr and a shared pointer to a node
  PluginMotors(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_);
  // Destructor
  ~PluginMotors();

private:
  // Shared pointer to the node
  rclcpp::Node::SharedPtr node_;
  // HardwarePtr object
  HardwarePtr hw_ptr_;

  // Function to write the robot speed
  void writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z);
  // Callback function for the cmd_vel topic
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  // Shared pointer to the cmd_vel subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

// Typedef for a shared pointer to a PluginMotors object
typedef std::shared_ptr<PluginMotors> MotorsPtr;
}  // namespace robotont
#endif