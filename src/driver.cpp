#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_odom.hpp"
#include "robotont_driver/plugin_motors.hpp"
#include "robotont_driver/plugin_led_module.hpp"

namespace robotont
{
  // Constructor for the Driver class
  Driver::Driver() : Node("driver_node")
  {
  }
  
  // Function to initialize the driver
  void Driver::initialize()
  {
    // Create a shared pointer of this node to allow ros functionality in subclasses
    auto node_ptr = shared_from_this();
    hw_ptr_ = std::make_shared<Hardware>(node_ptr);

    // Retrieve the parameter from the parameter server
    bool plugin_odom;
    bool plugin_motor;
    bool plugin_led_module;
    bool plugin_power_supply;
    bool plugin_range;
    try {
      plugin_odom = declare_parameter<bool>("plugin_odom", true);
      plugin_motor = declare_parameter<bool>("plugin_motor", true);
      plugin_led_module = declare_parameter<bool>("plugin_led_module", true);
      plugin_power_supply = declare_parameter<bool>("plugin_power_supply", true);
      plugin_range = declare_parameter<bool>("plugin_range", true);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "A provided plugin param was invalid");
      throw ex;
    }
    
    // Initialize plugins as defined by parameters
    if (plugin_odom) {
      odom_ptr_ = std::make_shared<PluginOdom>(node_ptr);
    }
    if (plugin_motor) {
      motor_ptr_ = std::make_shared<PluginMotors>(hw_ptr_, node_ptr);
    }
    if (plugin_led_module) {
      led_ptr_ = std::make_shared<PluginLedModule>(hw_ptr_, node_ptr);
    }
    
    // Create timer to read data from the robot 
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Driver::update_packet, this));
  }

  // Function to update the packet
  void Driver::update_packet()
  {
    hw_ptr_->get_packet(driver_packets);
    for (auto packet : driver_packets)
    {
      odom_ptr_->packetReceived(packet);
      for (auto arg : packet)
      {
        RCLCPP_DEBUG(this->get_logger(), "Received packet content: %s", arg.c_str());
      }
    }
  }

  // Destructor for the Driver class
  Driver::~Driver()
  {
  }

}