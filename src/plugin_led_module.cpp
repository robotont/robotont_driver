  #include "robotont_driver/plugin_led_module.hpp"

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
    // Subscribe to led_segment topic
    led_segment_sub_ = node_->create_subscription<robotont_msgs::msg::LedModuleSegment>("/led_segment",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)),
      std::bind(&PluginLedModule::segment_callback, this, std::placeholders::_1));  
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

  // Callback function to read data from led_pixel topic
  void PluginLedModule::pixel_callback(const robotont_msgs::msg::LedModulePixel::SharedPtr led_px_msg)
  {
    writePixel(led_px_msg->idx, led_px_msg->color.r, led_px_msg->color.g, led_px_msg->color.b);
  }

  void PluginLedModule::writeMode(const robotont_msgs::msg::LedModuleMode::SharedPtr led_mode_msg)
  {
    std::string packet;
    packet += "LM";
    packet += ":" + std::to_string(led_mode_msg->mode);
    for (auto &param : led_mode_msg->params)
    {
      packet += ":" + std::to_string(param);
    }
    packet += "\r\n";
    RCLCPP_INFO(node_->get_logger(), packet.c_str());

    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }

  void PluginLedModule::mode_callback(const robotont_msgs::msg::LedModuleMode::SharedPtr led_mode_msg)
  {
    writeMode(led_mode_msg);
  }

  void PluginLedModule::writeSegment(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg)
  {
    std::string packet;
    packet += "LS";
    packet += ":" + std::to_string(led_seg_msg->idx_start);
    packet += ":" + std::to_string(led_seg_msg->idx_end);
    for (auto &color : led_seg_msg->colors)
    {
      unsigned int color_combined = 0x000000;
      color_combined += (color.r << 16);
      color_combined += (color.g << 8);
      color_combined += (color.b);
      packet += ":" + std::to_string(color_combined);
    }
    packet += "\r\n";
    //RCLCPP_INFO(node_->get_logger(), packet.c_str());

    if (hw_ptr_)
    {
      hw_ptr_->subscriber_callback(packet);
    }
  }

  // Callback function to read data from led_segment topic
  void PluginLedModule::segment_callback(const robotont_msgs::msg::LedModuleSegment::SharedPtr led_seg_msg)
  {
    writeSegment(led_seg_msg);
  }

  } // namespace robotont
  
