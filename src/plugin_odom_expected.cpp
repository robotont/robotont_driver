#include "robotont_driver/plugin_odom_expected.h"

namespace robotont
{
  PluginOdomExpected::PluginOdomExpected(HardwarePtr hw_ptr, const std::string &name) : PluginBase(hw_ptr, name)
  {
    // Get frame names from parameter server
    std::string odom_frame;
    std::string odom_child_frame;
    nh_.param("odom/frame", odom_frame, std::string("odom_expected"));
    nh_.param("odom/child_frame", odom_child_frame, std::string("base_footprint"));

    // Set parent and child frame names for odom and its tf
    setFrameId(odom_frame);
    setChildFrameId(odom_child_frame);

    reset();

    // Initialize odom publisher
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_expected", 2);
  }

  PluginOdomExpected::~PluginOdomExpected()
  {
  }

  void PluginOdomExpected::setFrameId(const std::string &frame_id)
  {
    odom_msg_.header.frame_id = frame_id;
    odom_transform_.header.frame_id = frame_id;
  }

  void PluginOdomExpected::setChildFrameId(const std::string &child_frame_id)
  {
    odom_msg_.child_frame_id = child_frame_id;
    odom_transform_.child_frame_id = child_frame_id;
  }

  void PluginOdomExpected::reset()
  {
    odom_msg_.header.stamp = ros::Time::now();
    odom_msg_.pose.pose.position.x = 0;
    odom_msg_.pose.pose.position.y = 0;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.pose.orientation.x = 0;
    odom_msg_.pose.pose.orientation.y = 0;
    odom_msg_.pose.pose.orientation.z = 0;
    odom_msg_.pose.pose.orientation.w = 1;

    odom_transform_.header.stamp = odom_msg_.header.stamp;
    odom_transform_.transform.translation.x = 0;
    odom_transform_.transform.translation.y = 0;
    odom_transform_.transform.translation.z = 0;
    odom_transform_.transform.rotation.x = 0;
    odom_transform_.transform.rotation.y = 0;
    odom_transform_.transform.rotation.z = 0;
    odom_transform_.transform.rotation.w = 0;
  }

  void PluginOdomExpected::packetReceived(const RobotontPacket &packet)
  {
    if (packet.size() != 7 || packet[0] != "ODOM_EXPECTED")
    {
      return;
    }

    float pos_x, pos_y, ori_z, lin_vel_x, lin_vel_y, ang_vel_z;

    try
    {
      pos_x = std::stof(packet[1]);
      pos_y = std::stof(packet[2]);
      ori_z = std::stof(packet[3]);
      lin_vel_x = std::stof(packet[4]);
      lin_vel_y = std::stof(packet[5]);
      ang_vel_z = std::stof(packet[6]);
    }
    catch (std::exception e)
    {
      ROS_ERROR_THROTTLE(1, "Failed to parse odom packet: %s", e.what());
    }

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ori_z);

    odom_transform_.header.stamp = ros::Time::now();

    odom_transform_.transform.translation.x = pos_x;
    odom_transform_.transform.translation.y = pos_y;
    odom_transform_.transform.translation.z = 0.0;
    odom_transform_.transform.rotation = odom_quat;

    odom_msg_.header.stamp = ros::Time::now();

    odom_msg_.pose.pose.position.x = pos_x;
    odom_msg_.pose.pose.position.y = pos_y;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = odom_quat;

    odom_msg_.twist.twist.linear.x = lin_vel_x;
    odom_msg_.twist.twist.linear.y = lin_vel_y;
    odom_msg_.twist.twist.angular.z = ang_vel_z;

    publish();
  }

  void PluginOdomExpected::publish()
  {
    odom_pub_.publish(odom_msg_);
    odom_broadcaster_.sendTransform(odom_transform_);
  }

} // namespace robotont
