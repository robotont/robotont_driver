#ifndef PLUGIN_ODOM_EXPECTED_
#define PLUGIN_ODOM_EXPECTED_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "robotont_driver/plugin_base.h"

/**
 * \brief PluginOdomExpected class
 * This plugin parses the ODOM packet and translates it to ROS odom message.
 */
namespace robotont
{
  class PluginOdomExpected : public PluginBase
  {
  public:
    PluginOdomExpected(HardwarePtr hw_ptr, const std::string &name);
    ~PluginOdomExpected();

    void packetReceived(const RobotontPacket &packet);

  private:
    void reset();
    void setFrameId(const std::string &frame_id);

    void setChildFrameId(const std::string &child_frame_id);
    void publish();

    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped odom_transform_;
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
  };
} // namespace robotont
#endif
