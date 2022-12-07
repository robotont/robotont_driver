#include "robotont_driver/plugin_debug.h"

namespace robotont
{
    PluginDebug::PluginDebug(HardwarePtr hw_ptr, const std::string &name) : PluginBase(hw_ptr, name)
    {
        std::string odom_frame;
        std::string odom_child_frame;

        nh_.param("odom/frame", odom_frame, std::string("odom"));
        nh_.param("odom/child_frame", odom_child_frame, std::string("base_footprint"));
        reset();

        // Initialize debug pub and sub
        debug_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug_out", 2);
        debug_sub_ = nh_.subscribe("debug_in", 1, &PluginDebug::debug_in_callback, this);
    }

    PluginDebug::~PluginDebug()
    {
    }

    // Send data to board
    void PluginDebug::writeDebugData(const std_msgs::Float32MultiArray &debug_msg)
    {
        RobotontPacket packet;
        packet.push_back("DEBUG_IN");
        for (uint8_t i = 0; i < debug_msg.data.size(); i++)
        {
            packet.push_back(std::to_string(debug_msg.data[i]));
        }

        if (hw_ptr_)
        {
            hw_ptr_->writePacket(packet);
        }
    }

    void PluginDebug::debug_in_callback(const std_msgs::Float32MultiArray &debug_msg)
    {

        writeDebugData(debug_msg);
    }

    // Receive data from board
    void PluginDebug::reset()
    {
        debug_msg_.data = {0};
    }

    void PluginDebug::packetReceived(const RobotontPacket &packet)
    {
        if (packet[0] != "DEBUG_OUT")
        {
            return;
        }

        std::vector<float> data_out;
        for (uint8_t i = 1; i < packet.size(); i++)  // ! TODO elementide arv
        {
            try
            {
                data_out.push_back(std::stof(packet[i]));
            }
            catch (std::exception e)
            {
                ROS_ERROR_THROTTLE(1, "Failed to parse odom packet: %s", e.what());
            }
        }

        debug_msg_.data = data_out;
        publish();
    }

    void PluginDebug::publish()
    {
        debug_pub_.publish(debug_msg_);
    }

} // namespace robotont