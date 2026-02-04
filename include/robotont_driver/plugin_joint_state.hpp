// include/robotont_driver/plugin_joint_state.hpp:

#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

#ifndef JOINT_STATE_HPP
#define JOINT_STATE_HPP

namespace robotont
{
// PluginJointState class
class PluginJointState
{
public:
  // Constructor that takes a shared pointer to a node
  PluginJointState(rclcpp::Node::SharedPtr node_);

  // Destructor
  ~PluginJointState();

  // Function to reset the joint state message values
  void reset();

  // Function to receive the packet and transform it into a joint state message
  void packetReceived(const std::vector<std::string>& packet);

  // Function to publish the joint state message
  void publish();

private:
  // Shared pointer to the node
  rclcpp::Node::SharedPtr node_;

  // Unique pointer to the joint state message
  sensor_msgs::msg::JointState::UniquePtr joint_state_msg_;

  // Shared pointer to the joint state publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Weak pointer to the driver node
  rclcpp::Node::WeakPtr weak_node_;
};

// Typedef for a shared pointer to a PluginJointState object
typedef std::shared_ptr<PluginJointState> JointStatePtr;

} // namespace robotont

#endif