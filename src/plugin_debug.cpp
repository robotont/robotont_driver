#include "robotont_driver/plugin_debug.h"

namespace robotont
{
    PluginDebug::PluginDebug(HardwarePtr hw_ptr, const std::string &name) : PluginBase(hw_ptr, name)
    {
        debug_sub_ = nh_.subscribe("cmd_vel", 1, &PluginDebug::debug_in_callback, this);
    }

    PluginDebug::~PluginDebug()
    {
    }

    void PluginDebug::writeDebugData(float data[], uint8_t size)
    {
        RobotontPacket packet;
        packet.push_back("DEBUG_IN");
        for (uint8_t i = 0; i < size; i++)
        {
            packet.push_back(std::to_string(data[i]));
        }

        if (hw_ptr_)
        {
            hw_ptr_->writePacket(packet);
        }
    }

    void PluginDebug::debug_in_callback(const std_msgs::Float32MultiArray &debug_msg)
    {
        writeDebugData(debug_msg.data, sizeof(debug_msg.data)); // ! HELP
    }

} // namespace robotont