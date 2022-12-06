#ifndef PLUGIN_DEBUG_
#define PLUGIN_DEBUG_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> 
#include "robotont_driver/plugin_base.h"

/**
 * \brief PluginDebug class
 * This plugin subscribes to debug_in topic and relays any debug data as array to the mbed board.
 */
namespace robotont
{
    class PluginDebug : public PluginBase
    {
    public:
        PluginDebug(HardwarePtr hw_ptr, const std::string &name);
        ~PluginDebug();

    private:
        void writeDebugData(float data[], uint8_t size);
        void debug_in_callback(const std_msgs::Float32MultiArray& debug_msg);

        ros::Subscriber debug_sub_;
    };
} // namespace robotont

#endif