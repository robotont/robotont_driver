#ifndef DRIVER_
#define DRIVER_

#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_base.hpp"
#include "robotont_driver/plugin_odom.hpp"
#include "robotont_driver/plugin_motors.hpp"
#include "robotont_driver/plugin_led_module.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"
#include "msg_converters/converters.hpp"
#include "robotont_driver/hardware.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace robotont
{
// Driver class that inherits from rclcpp::Node
class Driver : public rclcpp::Node
{    
public:
  // Constructor
  Driver();
  // Function to initialize the driver
  void initialize();
  // Function to update the packet
  void update_packet();
  // Destructor
  ~Driver();

  // Vector to store driver packets
  std::vector<std::vector<std::string>> driver_packets;

private:
  // Function to update the driver
  void update();

  // Pointer to the Hardware object
  HardwarePtr hw_ptr_;
  // Pointer to the Odom object
  OdomPtr odom_ptr_;
  // Pointer to the Motors object
  MotorsPtr motor_ptr_;
  LedModulePtr led_ptr_;

  // Vector to store hardware packets
  std::vector<std::string> hardware_packet_;
  // Vector to store PluginBasePtr objects
  std::vector<PluginBasePtr> plugins_;
  // Shared pointer to the timer
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotont
#endif