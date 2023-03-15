#include "robotont_driver/plugin_debug_out.h"

namespace robotont
{
  PluginDebugOut::PluginDebugOut(HardwarePtr hw_ptr, const std::string &name) : PluginBase(hw_ptr, name)
  {
    // Get frame names from parameter server
    // std::string debug_frame;
    // std::string odom_child_frame;
    // nh_.param("odom/frame", debug_frame, std::string("debug_out"));
    // nh_.param("odom/child_frame", odom_child_frame, std::string("base_footprint"));
    reset();

    debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug_out", 10);
  }

  PluginDebugOut::~PluginDebugOut()
  {
  }

  void PluginDebugOut::reset()
  {
    debug_msg_.data.clear();
  }

  void PluginDebugOut::packetReceived(const RobotontPacket &packet)
  {
    if (packet[0] != "DEBUG_OUT")
    {
      return;
    }

    reset();
    try
    {
      for (uint8_t i = 0; i < packet.size() - 1; i++)
      {
        debug_msg_.data.push_back(std::stof(packet[i + 1]));
      }
    }
    catch (std::exception e)
    {
      ROS_ERROR_THROTTLE(1, "Failed to parse odom packet: %s", e.what());
    }
    publish();
  }

  void PluginDebugOut::publish()
  {
    debug_pub_.publish(debug_msg_);
  }

} // namespace robotont
