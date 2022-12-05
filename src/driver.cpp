#include "robotont_driver/driver.h"
#include "robotont_driver/plugin_motors.h"
#include "robotont_driver/plugin_odom.h"
#include "robotont_driver/plugin_odom_expected.h"
#include "robotont_driver/plugin_power_supply.h"
#include "robotont_driver/plugin_range.h"
#include "robotont_driver/plugin_led_module.h"

namespace robotont
{
  Driver::Driver()
  {
    // Create a hardware instance
    hw_ptr_ = std::make_shared<Hardware>();

    // Initialize builtin plugins
    plugins_.emplace_back(std::make_shared<PluginMotors>(hw_ptr_, "Motors"));
    plugins_.emplace_back(std::make_shared<PluginOdom>(hw_ptr_, "Odometry"));
    plugins_.emplace_back(std::make_shared<PluginOdomExpected>(hw_ptr_, "OdometryExpected"));
    plugins_.emplace_back(std::make_shared<PluginPowerSupply>(hw_ptr_, "PowerSupply"));
    plugins_.emplace_back(std::make_shared<PluginRange>(hw_ptr_, "Range sensor"));
    plugins_.emplace_back(std::make_shared<PluginLedModule>(hw_ptr_, "LedModulePixel"));

    // Here we load all the possible plugins
    for (auto plugin : plugins_)
    {
      if (plugin)
      {
        ROS_INFO("Initializing plugin: '%s'.", plugin->getName().c_str());
        plugin->initialize();
      }
    }

    // Create a timer to periodically read data from the robot
    timer_ = nh_.createTimer(ros::Duration(0.01), &Driver::update, this);
  }

  Driver::~Driver()
  {
  }

  void Driver::update(const ros::TimerEvent& event)
  {
    // Poll serial
    RobotontPacket packet;
    if (hw_ptr_)
    {
      // Check if a packet is arrived from the robot
      if(hw_ptr_->readPacket(packet))
      {
        for (auto arg : packet)
        {
          ROS_DEBUG_STREAM(arg);
        }
        // We have received the complete packet
        // Process the packet in each loaded plugin
        for (auto& plugin : plugins_)
        {
          if(plugin)
          {
            plugin->packetReceived(packet);
          }
        }
      }
    }
  }
}  // namespace robotont
