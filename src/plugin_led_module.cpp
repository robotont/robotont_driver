  /*#include "robotont_driver/plugin_led_module.hpp"

  using namespace std::chrono_literals;

  namespace robotont
  {
  PluginLedModule::PluginLedModule(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_) : hw_ptr_(hw_ptr), node_(node_)
  {
    RCLCPP_INFO(node_->get_logger(), "Robotont LED module is starting...");

    // Subscribe to led_pixel topic
    led_pixel_sub_ = node_->create_subscription<robotont_msgs::msg::LedModulePixel>("/led_pixel", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
      std::bind(&PluginLedModule::pixel_callback, this, std::placeholders::_1));
    // Subscribe to led_mode topic
    led_mode_sub_ = node_->create_subscription<robotont_msgs::msg::LedModuleMode>("/led_mode", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
      std::bind(&PluginLedModule::mode_callback, this, std::placeholders::_1));
  }

  PluginLedModule::~PluginLedModule()
  {
  }

  void PluginLedModule::writePixel(unsigned int idx, uint8_t r, uint8_t g, uint8_t b)
  {
    //RCLCPP_INFO(node_->get_logger(), ("Sent LD:"+std::to_string(idx)+":"+std::to_string(r)+":"+std::to_string(g)+":"+std::to_string(b)).c_str());
    std::string packet = "LD:"+std::to_string(idx)+":"+std::to_string(r)+":"+std::to_string(g)+":"+std::to_string(b)+"\r\n";
    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }

  // Callback function to read data from cmd_vel topic
  void PluginLedModule::pixel_callback(const robotont_msgs::msg::LedModulePixel::SharedPtr led_px_msg)
  {
    writePixel(led_px_msg->idx, led_px_msg->color.r, led_px_msg->color.g, led_px_msg->color.b);
  }

  void PluginLedModule::writeMode(uint8_t mode, uint8_t r, uint8_t g, uint8_t b)
  {
    //RCLCPP_INFO(node_->get_logger(), ("Sent LM:"+std::to_string(mode)+":"+std::to_string(r)+":"+std::to_string(g)+":"+std::to_string(b)).c_str());
    std::string packet = "LM:"+std::to_string(mode)+":"+std::to_string(r)+":"+std::to_string(g)+":"+std::to_string(b)+"\r\n";
    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }

  // Callback function to read data from cmd_vel topic
  void PluginLedModule::mode_callback(const robotont_msgs::msg::LedModuleMode::SharedPtr led_mode_msg)
  {
    writeMode(led_mode_msg->mode, led_mode_msg->color.r, led_mode_msg->color.g, led_mode_msg->color.b);
  }

  /*void PluginLedModule::writeSegment(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg)
  {
    RCLCPP_INFO(node_->get_logger(), ("Sent LS:"+std::to_string(led_seg_msg.idx_start)+":...").c_str());
    std::string packet = "LS:"+std::to_string(idx)+":"+std::to_string(r)+":"+std::to_string(g)+":"+std::to_string(b)+"\r\n";
    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }

  // Callback function to read data from cmd_vel topic
  void PluginLedModule::segment_callback(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg)
  {
    writeSegment(led_px_msg->idx, led_px_msg->color.r, led_px_msg->color.g, led_px_msg->color.b);
  }*/


  /*
  // Function to create the string packet from cmd_vel topic and send it to serial
  void PluginLedModule::writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z)
  {
    std::string packet = "RS:"+std::to_string(lin_vel_x)+":"+std::to_string(lin_vel_y)+":"+std::to_string(ang_vel_z)+"\r\n";
    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }*/


  //} // namespace robotont
  
