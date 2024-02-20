#ifndef HARDWARE_
#define HARDWARE_
#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/plugin_odom.hpp"
#include "io_context/io_context.hpp"
#include "serial_driver/serial_driver.hpp"
#include "msg_converters/converters.hpp"

using std_msgs::msg::UInt8MultiArray;

namespace robotont
{
// Typedef for a vector of strings representing a RobotontPacket
typedef std::vector<std::string> RobotontPacket;

// Hardware class
class Hardware
{
public:
  // Constructor that takes a shared pointer to a node
  Hardware(rclcpp::Node::SharedPtr node);
  // Destructor
  ~Hardware();

  // Function to read from the hardware
  void read();
  // Callback function for the cmd_vel topic
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  // Function to get parameters
  void get_params();
  // Callback function for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
  // Callback function for sending a raw serial message
  void subscriber_callback(std::string send_packet);
  // Function to get a packet
  void get_packet(std::vector<RobotontPacket> & driver_packets);

private:
  // Function to check the serial port
  void checkSerialPort();

  // Unique pointer to the IoContext object
  std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
  // String to store the device name
  std::string m_device_name{};
  // Unique pointer to the SerialPortConfig object
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_device_config;
  // Unique pointer to the SerialDriver object
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_serial_driver;
  // Shared pointer to the publisher
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr m_publisher;
  // Shared pointer to the subscriber
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;
  // Shared pointer to the node
  rclcpp::Node::SharedPtr node_;

  // RobotontPacket object
  RobotontPacket packet_;
  // Vector to store RobotontPacket objects
  std::vector<RobotontPacket> packets_;
  // Mutex for thread safety
  std::mutex mutex_;
  // String to store the packet
  std::string packet_buffer_;
  // Shared pointer to the timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Shared pointer to the serial watchdog timer
  rclcpp::TimerBase::SharedPtr serial_wdt_;
  // Boolean to indicate if a reconnect is requested
  bool reconnect_requested_;
}; //class hardware

// Typedef for a shared pointer to a Hardware object
typedef std::shared_ptr<Hardware> HardwarePtr;

}  // namespace robotont

#endif