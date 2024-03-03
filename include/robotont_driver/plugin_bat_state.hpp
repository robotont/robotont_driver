#ifndef PLUGIN_BAT_STATE_HPP
#define PLUGIN_BAT_STATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <sensor_msgs/msg/battery_state.hpp>

namespace robotont
{
// PluginBatState class
class PluginBatState
{
public:
  // Constructor that takes a shared pointer to a node
  PluginBatState(rclcpp::Node::SharedPtr node_);

  // Destructor
  ~PluginBatState();

  // Function to reset the battery state message values
  void reset();

  // Function to receive the packet and transform it into a battery state message
  void packetReceived(const std::vector<std::string>& packet);

  // Function to publish the battery state
  void publish();

  // Function to update the battery state
  void update(float voltage, float current);

private:
  // Shared pointer to the node
  rclcpp::Node::SharedPtr node_;

  // Unique pointer to the battery state message
  std::unique_ptr<sensor_msgs::msg::BatteryState> battery_state_msg_;

  // Shared pointer to the battery state publisher
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
};

// Typedef for a shared pointer to a PluginBatState object
typedef std::shared_ptr<PluginBatState> BatStatePtr;

} // namespace robotont

#endif
