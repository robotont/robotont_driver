#include "robotont_driver/hardware.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robotont_driver/plugin_odom.hpp"

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace robotont
{
Hardware::Hardware(rclcpp::Node::SharedPtr node): 
    m_owned_ctx{new drivers::common::IoContext()},
    m_serial_driver{new drivers::serial_driver::SerialDriver(*m_owned_ctx)},
    node_(node),
    last_receive_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(node_->get_logger(), "Robotont driver is starting...");

  // Get parameters 
  get_params();
  
  // Initialise and open serial port
  try {
    m_serial_driver->init_port(m_device_name, *m_device_config);
    if (!m_serial_driver->port()->is_open()) {
      m_serial_driver->port()->open();
      m_serial_driver->port()->async_receive(
        std::bind(&Hardware::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Error creating serial port: %s - %s",
      m_device_name.c_str(), ex.what());
  }

  // Create a watchdog timer for serial port monitoring
  serial_wdt_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&Hardware::checkSerialPort, this));
  
  // Create a watchdog timer for receive timeout monitoring (detects communication failure)
  receive_wdt_ = node_->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Hardware::checkReceiveTimeout, this));


  RCLCPP_INFO(node_->get_logger(), "Hardware interface is ready");
}

//Check if serial port is open, if it's closed then reopen the port
void Hardware::checkSerialPort()
{
    RCLCPP_DEBUG(node_->get_logger(), "Checking port...");
    
    try{
      if (!m_serial_driver->port()->is_open()) {
      RCLCPP_DEBUG(node_->get_logger(), "Port closed, reopening...");
        m_serial_driver->port()->open();
        m_serial_driver->port()->async_receive(
          std::bind(&Hardware::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        RCLCPP_DEBUG(node_->get_logger(), "Port open.");
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "Error creating serial port: %s - %s",
        m_device_name.c_str(), ex.what());
    }
}

// Check if we've received data recently, force reconnect if silent for too long
void Hardware::checkReceiveTimeout()
{
  auto now = std::chrono::steady_clock::now();
  auto last = last_receive_time_.load();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
  
  // If no data received for 5 seconds and port is "open", something is wrong
  constexpr int64_t RECEIVE_TIMEOUT_MS = 5000;
  
  if (elapsed_ms > RECEIVE_TIMEOUT_MS && m_serial_driver->port() && m_serial_driver->port()->is_open()) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "No data received for %ld ms - forcing port reconnect", elapsed_ms);
    try {
      m_serial_driver->port()->close();
      // The checkSerialPort watchdog will handle reopening
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Error closing port: %s", ex.what());
    }
  }
}

// Return list of driver packets 
void Hardware::get_packet(std::vector<RobotontPacket> &  driver_packets)
{
  mutex_.lock();
  driver_packets = std::move(packets_);
  mutex_.unlock();
}

// Callback function for reading from serial
void Hardware::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
{
  last_receive_time_.store(std::chrono::steady_clock::now());

  mutex_.lock();
  packet_buffer_.append(std::string(buffer.begin(), buffer.begin() + bytes_transferred));

  // Trim leading line endings
  size_t packet_beg_pos = packet_buffer_.find_first_not_of("\r\n");
  if (packet_beg_pos == std::string::npos)
  {
    packet_buffer_ = "";
    mutex_.unlock();
    return;
  }
  packet_buffer_.erase(0, packet_beg_pos);

  // *** Parse ALL complete packets in the buffer, not just one ***
  size_t packet_end_pos;
  while ((packet_end_pos = packet_buffer_.find_first_of("\r\n")) != std::string::npos)
  {
    if (packet_end_pos > 2)
    {
      std::string packet_str = packet_buffer_.substr(0, packet_end_pos);

      std::stringstream packet_ss(packet_str);
      std::string arg;
      packet_.clear();
      while (std::getline(packet_ss, arg, ':'))
      {
        packet_.push_back(arg);
      }
      packets_.push_back(packet_);
    }
    // Erase past the delimiter
    packet_buffer_.erase(0, packet_end_pos + 1);

    // Trim any additional leading newlines before next iteration
    size_t next_beg = packet_buffer_.find_first_not_of("\r\n");
    if (next_beg == std::string::npos) { packet_buffer_ = ""; break; }
    packet_buffer_.erase(0, next_beg);
  }

  mutex_.unlock();
}

// Callback function for sending data to serial port
void Hardware::subscriber_callback(std::string send_packet)
{
  RCLCPP_DEBUG(node_->get_logger(), "Subscriber_callback");
  std::vector<uint8_t> vec(send_packet.begin(), send_packet.end());
  m_serial_driver->port()->async_send(vec);
}

Hardware::~Hardware()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
  }
}

// Function to define serial parameters
void Hardware::get_params()
{
  uint32_t baud_rate{};
  auto fc = drivers::serial_driver::FlowControl::NONE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;

  // Device name parameter
  if (!node_->has_parameter("device_name")) {
    node_->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
  }
  m_device_name = node_->get_parameter("device_name").as_string();

  // Baud rate parameter
  if (!node_->has_parameter("baud_rate")) {
    node_->declare_parameter<int>("baud_rate", 115200);
  }
  baud_rate = node_->get_parameter("baud_rate").as_int();

  // Flow control parameter
  if (!node_->has_parameter("flow_control")) {
    node_->declare_parameter<std::string>("flow_control", "none");
  }
  const auto fc_string = node_->get_parameter("flow_control").as_string();
  if (fc_string == "none") {
    fc = drivers::serial_driver::FlowControl::NONE;
  } else if (fc_string == "hardware") {
    fc = drivers::serial_driver::FlowControl::HARDWARE;
  } else if (fc_string == "software") {
    fc = drivers::serial_driver::FlowControl::SOFTWARE;
  } else {
    throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
  }

  // Parity parameter
  if (!node_->has_parameter("parity")) {
    node_->declare_parameter<std::string>("parity", "none");
  }
  const auto pt_string = node_->get_parameter("parity").as_string();
  if (pt_string == "none") {
    pt = drivers::serial_driver::Parity::NONE;
  } else if (pt_string == "odd") {
    pt = drivers::serial_driver::Parity::ODD;
  } else if (pt_string == "even") {
    pt = drivers::serial_driver::Parity::EVEN;
  } else {
    throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
  }

  // Stop bits parameter
  if (!node_->has_parameter("stop_bits")) {
    node_->declare_parameter<std::string>("stop_bits", "1");
  }
  const auto sb_string = node_->get_parameter("stop_bits").as_string();
  if (sb_string == "1" || sb_string == "1.0" || sb_string == "one") {
    sb = drivers::serial_driver::StopBits::ONE;
  } else if (sb_string == "1.5" || sb_string == "one_point_five") {
    sb = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
  } else if (sb_string == "2" || sb_string == "2.0" || sb_string == "two") {
    sb = drivers::serial_driver::StopBits::TWO;
  } else {
    throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
  }

  m_device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}
}
