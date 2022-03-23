#include "robotont_driver/hardware.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace drivers
{
namespace serial_driver
{

Hardware::Hardware() : Node("hardware_node"),
    m_owned_ctx{new IoContext()},
    m_serial_driver{new SerialDriver(*m_owned_ctx)}
{
  RCLCPP_INFO(this->get_logger(), "Robotont driver is starting...");

  // Get parameters 
  get_params();
  // Initialize
  initialize();
}

void Hardware::initialize()
{

  // Create Publisher
  m_publisher = this->create_publisher<UInt8MultiArray>(
    "serial_read", rclcpp::QoS{100});
  
  try {
    m_serial_driver->init_port(m_device_name, *m_device_config);
    if (!m_serial_driver->port()->is_open()) {
      m_serial_driver->port()->open();
      m_serial_driver->port()->async_receive(
        std::bind(&Hardware::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s",
      m_device_name.c_str(), ex.what());
  }

  // Create Subscriber
  //auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
  //auto callback = std::bind(&Hardware::subscriber_callback, this, std::placeholders::_1);
  //m_subscriber = this->create_subscription<UInt8MultiArray>("cmd_vel", qos, callback);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
    std::bind(&Hardware::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Serial driver is running");
}

void Hardware::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  RCLCPP_INFO(this->get_logger(), "got vel cmd");
}

void Hardware::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
{
  UInt8MultiArray out;
  UInt8MultiArray out2;
  drivers::common::to_msg(buffer, out, bytes_transferred);
  m_publisher->publish(out);
  m_publisher->publish(out2);
}


void Hardware::subscriber_callback(const UInt8MultiArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Subscriber_callback");

  /*
  std::vector<uint8_t> out;
  drivers::common::from_msg(msg, out);
  m_serial_driver->port()->async_send(out);
  RCLCPP_INFO(this->get_logger(), "Sent data %s", out);
  */
}

Hardware::~Hardware()
{
}

void Hardware::get_params()
{
  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    m_device_name = declare_parameter<std::string>("device_name", "/dev/ttyACM1");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 115200);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "none");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
              "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "none");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{
              "The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "1");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{
              "The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  m_device_config = std::make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
}
}  // namespace serial_driver
}  // namespace drivers