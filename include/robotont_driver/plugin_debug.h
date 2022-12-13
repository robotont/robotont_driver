#ifndef PLUGIN_DEBUG_
#define PLUGIN_DEBUG_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robotont_driver/plugin_base.h"

#define DEBUG_ARGS 10

/**
 * \brief PluginDebug class
 * This plugin subscribes to debug_in topic and relays any debug data as array to the mbed board.
 * 
 * Plugin reads data from board in format "DEBUG_OUT:arg1,...argN\r\n", where amount of args defined in
 * the DEBUG_MAX_ARGS. Next step, publishes received arrays as std_msgs/Float32MultiArray message.
 * Also, plugin publishes data in the same format to the board.
 */
namespace robotont
{
    class PluginDebug : public PluginBase
    {
    public:
        PluginDebug(HardwarePtr hw_ptr, const std::string &name);
        ~PluginDebug();

        void packetReceived(const RobotontPacket &packet);

    private:
        // Send data to board
        void writeDebugData(const std_msgs::Float32MultiArray &debug_msg);
        void debug_in_callback(const std_msgs::Float32MultiArray &debug_msg);
        // Receive data from board
        void reset();
        void publish();

        ros::NodeHandle nh_;
        ros::Subscriber debug_sub_;
        ros::Publisher debug_pub_;
        std_msgs::Float32MultiArray debug_msg_;
    };
} // namespace robotont
#endif