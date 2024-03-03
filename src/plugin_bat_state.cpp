#include "robotont_driver/plugin_bat_state.hpp"

namespace robotont
{
PluginBatState::PluginBatState(rclcpp::Node::SharedPtr node_) : node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont Bat State is starting...");
  // Create messages
  battery_state_msg_ = std::make_unique<sensor_msgs::msg::BatteryState>();

  // Initialize messages
  reset();

  // Initialize batstate publisher
  battery_state_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 1);
}

PluginBatState::~PluginBatState()
{
}

void PluginBatState::packetReceived(const std::vector<std::string>& packet)
{
  if (packet.size() != 5 || packet[0] != "BAT")
  {
    return;
  }

  float voltage, current;

  try
  {
    voltage = std::stof(packet[2]);
    current = std::stof(packet[4]);

  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "BATSTATE error reading packet: %s", e.what());
    return;
  }

  update(voltage, current);
}

void PluginBatState::publish()
{
  if (battery_state_pub_)
  {
    battery_state_pub_->publish(*battery_state_msg_);
  }
}

void PluginBatState::update(float voltage, float current)
{
  battery_state_msg_->header.stamp = node_->now();
  battery_state_msg_->voltage = voltage;
  battery_state_msg_->current = current;

  publish(); // Optionally publish the updated battery state immediately
}

void PluginBatState::reset()
{
  battery_state_msg_->header.stamp = node_->now();
  battery_state_msg_->voltage = 0;
  battery_state_msg_->current = 0;
}

} // namespace robotont
