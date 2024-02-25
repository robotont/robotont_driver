#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include "robotont_driver/plugin_base.hpp"

#include "robotont_msgs/msg/led_module_pixel.hpp"
#include "robotont_msgs/msg/led_module_mode.hpp" 
#include "robotont_msgs/msg/led_module_segment.hpp"

#ifndef LED_HPP
#define LED_HPP


using namespace std::chrono_literals;
/**
 * \brief PluginLedModule class
 * This plugin subscribes to led_pixel and led_segment topics, which can be used to set the led module color.
 */
namespace robotont
{
class PluginLedModule
{
public:
  PluginLedModule(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_);
  ~PluginLedModule();

private:
  rclcpp::Node::SharedPtr node_;
  HardwarePtr hw_ptr_;

  void writePixel(unsigned int idx, uint8_t r, uint8_t g, uint8_t b);
  void pixel_callback(const robotont_msgs::msg::LedModulePixel::SharedPtr led_px_msg);
  void writeMode(uint8_t mode, uint8_t r, uint8_t g, uint8_t b);
  void mode_callback(const robotont_msgs::msg::LedModuleMode::SharedPtr led_mode_msg);
  void writeSegment(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg);
  void segment_callback(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg);

  rclcpp::Subscription<robotont_msgs::msg::LedModulePixel>::SharedPtr led_pixel_sub_; //TODO: fix issue with not being able to access RobotontPacket within the same namespace. When done, also update other classes to use RobotontPacket.
  rclcpp::Subscription<robotont_msgs::msg::LedModuleMode>::SharedPtr led_mode_sub_;
  rclcpp::Subscription<robotont_msgs::msg::LedModuleSegment>::SharedPtr led_segment_sub_;
};

typedef std::shared_ptr<PluginLedModule> LedModulePtr;
}  // namespace robotont
#endif