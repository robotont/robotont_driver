#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_odom.hpp"
#include "robotont_driver/plugin_motors.hpp"
#include "robotont_driver/plugin_led_module.hpp"
#include "robotont_driver/plugin_bat_state.hpp"

namespace robotont
{
  // Constructor for the Driver class
  Driver::Driver() : Node("driver_node", rclcpp::NodeOptions().enable_logger_service(true))
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
    bool plugin_battery_state;
    bool plugin_range;
    bool plugin_bat_state;
    try {
      if (!this->has_parameter("plugin_odom")) {
        plugin_odom = this->declare_parameter<bool>("plugin_odom", true);
      } else {
        plugin_odom = this->get_parameter("plugin_odom").as_bool();
      }

      if (!this->has_parameter("plugin_motor")) {
        plugin_motor = this->declare_parameter<bool>("plugin_motor", true);
      } else {
        plugin_motor = this->get_parameter("plugin_motor").as_bool();
      }

      if (!this->has_parameter("plugin_led_module")) {
        plugin_led_module = this->declare_parameter<bool>("plugin_led_module", true);
      } else {
        plugin_led_module = this->get_parameter("plugin_led_module").as_bool();
      }

      if (!this->has_parameter("plugin_battery_state")) {
        plugin_battery_state = this->declare_parameter<bool>("plugin_battery_state", true);
      } else {
        plugin_battery_state = this->get_parameter("plugin_power_supply").as_bool();
      }

      if (!this->has_parameter("plugin_range")) {
        plugin_range = this->declare_parameter<bool>("plugin_range", true);
      } else {
        plugin_range = this->get_parameter("plugin_range").as_bool();
      }

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
      //led_ptr_ = std::make_shared<PluginLedModule>(hw_ptr_, node_ptr);
    }
    if (plugin_battery_state) {
      //create pointer for battery state plugin
    }
    if (plugin_range) {
      //create pointer for range plugin
    }
    if (plugin_bat_state){
      bat_state_ptr_ = std::make_shared<PluginBatState>(node_ptr);
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
      bat_state_ptr_->packetReceived(packet);
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
