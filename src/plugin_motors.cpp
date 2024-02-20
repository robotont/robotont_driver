#include "robotont_driver/plugin_motors.hpp"

using namespace std::chrono_literals;

namespace robotont
{
// Constructor for the PluginMotors class
PluginMotors::PluginMotors(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_) : hw_ptr_(hw_ptr), node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont motors are starting...");

  // Subscribe to command velocity topic
   cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
    std::bind(&PluginMotors::cmd_vel_callback, this, std::placeholders::_1));
}

// Destructor for the PluginMotors class
PluginMotors::~PluginMotors()
{
}

// Callback function to read data from cmd_vel topic
void PluginMotors::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  writeRobotSpeed(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
  RCLCPP_DEBUG(node_->get_logger(), "Command sent via serial");
}

// Function to create the string packet from cmd_vel topic and send it to serial
void PluginMotors::writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  std::string packet = "RS:"+std::to_string(lin_vel_x)+":"+std::to_string(lin_vel_y)+":"+std::to_string(ang_vel_z)+"\r\n";
  if (hw_ptr_)
  {
    hw_ptr_->subscriber_callback(packet);
  }
}

} // namespace robotont