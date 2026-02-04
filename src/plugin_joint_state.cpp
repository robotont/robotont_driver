// src/plugin_joint_state.cpp:

#include "robotont_driver/plugin_joint_state.hpp"

using namespace std::chrono_literals;

namespace robotont
{
// Constructor for the PluginJointState class
PluginJointState::PluginJointState(rclcpp::Node::SharedPtr node_) : node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont joint state publisher plugin is starting...");

  // Declare parameters
  node_->declare_parameter<std::string>("rim_left_joint", "rim_left_joint");
  node_->declare_parameter<std::string>("rim_back_joint", "rim_back_joint");
  node_->declare_parameter<std::string>("rim_right_joint", "rim_right_joint");

  // Get parameters
  std::string rim_left = node_->get_parameter("rim_left_joint").as_string();
  std::string rim_back = node_->get_parameter("rim_back_joint").as_string();
  std::string rim_right = node_->get_parameter("rim_right_joint").as_string();
  std::string frame_prefix = node_->get_parameter("frame_prefix").as_string();

  RCLCPP_INFO(node_->get_logger(), "Using frame_prefix: %s", frame_prefix.c_str());

  // Construct full joint names
  std::string joint_left = frame_prefix.empty() ? rim_left : frame_prefix + "/" + rim_left;
  std::string joint_back = frame_prefix.empty() ? rim_back : frame_prefix + "/" + rim_back;
  std::string joint_right = frame_prefix.empty() ? rim_right : frame_prefix + "/" + rim_right;

  // Create message
  joint_state_msg_ = std::make_unique<sensor_msgs::msg::JointState>();

  // Set joint names (order: left, back, right)
  joint_state_msg_->name = {joint_left, joint_back, joint_right};
  joint_state_msg_->position.resize(3, 0.0);
  joint_state_msg_->velocity.resize(3, 0.0);
  joint_state_msg_->effort.resize(3, 0.0);

  // Initialize message
  reset();

  // Initialize joint state publisher
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  RCLCPP_INFO(node_->get_logger(), "Publishing joint states for: %s, %s, %s",
              joint_left.c_str(), joint_back.c_str(), joint_right.c_str());
}

// Destructor for the PluginJointState class
PluginJointState::~PluginJointState()
{
}

// Create JointState message from the data received from serial port
// Packet format: JS pos0 pos1 pos2 vel0 vel1 vel2 eff0 eff1 eff2
// Order: left, back, right
void PluginJointState::packetReceived(const std::vector<std::string>& packet)
{
  if (packet.size() != 10 || packet[0] != "JS")
  {
    return;
  }

  float pos_left, pos_back, pos_right;
  float vel_left, vel_back, vel_right;
  float eff_left, eff_back, eff_right;

  try
  {
    pos_left = std::stof(packet[1]);
    pos_back = std::stof(packet[2]);
    pos_right = std::stof(packet[3]);
    vel_left = std::stof(packet[4]);
    vel_back = std::stof(packet[5]);
    vel_right = std::stof(packet[6]);
    eff_left = std::stof(packet[7]);
    eff_back = std::stof(packet[8]);
    eff_right = std::stof(packet[9]);
  }
  catch (std::exception e)
  {
    RCLCPP_ERROR(node_->get_logger(), "JS error reading packet");
    return;
  }

  joint_state_msg_->header.stamp = node_->now();
  joint_state_msg_->position[0] = pos_left;
  joint_state_msg_->position[1] = pos_back;
  joint_state_msg_->position[2] = pos_right;
  joint_state_msg_->velocity[0] = vel_left;
  joint_state_msg_->velocity[1] = vel_back;
  joint_state_msg_->velocity[2] = vel_right;
  joint_state_msg_->effort[0] = eff_left;
  joint_state_msg_->effort[1] = eff_back;
  joint_state_msg_->effort[2] = eff_right;

  publish();
}

// Publish the joint state message to joint_states topic
void PluginJointState::publish()
{
  if (joint_state_pub_)
  {
    (joint_state_pub_->publish)(*joint_state_msg_);
  }
}

// Clear joint state message
void PluginJointState::reset()
{
  joint_state_msg_->header.stamp = node_->now();
  joint_state_msg_->position[0] = 0;
  joint_state_msg_->position[1] = 0;
  joint_state_msg_->position[2] = 0;
  joint_state_msg_->velocity[0] = 0;
  joint_state_msg_->velocity[1] = 0;
  joint_state_msg_->velocity[2] = 0;
  joint_state_msg_->effort[0] = 0;
  joint_state_msg_->effort[1] = 0;
  joint_state_msg_->effort[2] = 0;
}

} // namespace robotont