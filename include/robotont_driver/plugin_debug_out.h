#ifndef PLUGIN_DEBUG_OUT_
#define PLUGIN_DEBUG_OUT_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robotont_driver/plugin_base.h"

/**
 * \brief DebugOut class
 */
namespace robotont
{
class PluginDebugOut : public PluginBase
{
public:
  PluginDebugOut(HardwarePtr hw_ptr, const std::string& name);
  ~PluginDebugOut();

  void packetReceived(const RobotontPacket& packet);

private:
  void reset();
  void publish();

  ros::NodeHandle nh_;
  ros::Publisher debug_pub_;
  std_msgs::Float32MultiArray debug_msg_;
};
}  // namespace robotont
#endif
