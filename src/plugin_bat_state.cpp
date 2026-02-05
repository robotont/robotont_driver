#include "robotont_driver/plugin_bat_state.hpp"
#include <algorithm>

namespace robotont
{

constexpr float BATTERY_VOLTAGE_FULL = 21.0f;  // 5 cells * 4.2V
constexpr float BATTERY_VOLTAGE_EMPTY = 15.0f; // 5 cells * 3.0V

PluginBatState::PluginBatState(rclcpp::Node::SharedPtr node_) : node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont Bat State is starting...");
  // Create messages
  battery_state_msg_ = std::make_unique<sensor_msgs::msg::BatteryState>();

  // Initialize messages
  reset();

  // Initialize batstate publisher
  battery_state_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 1);
}

PluginBatState::~PluginBatState()
{
}

void PluginBatState::packetReceived(const std::vector<std::string>& packet)
{
  if (packet.size() != 12 || packet[0] != "BATSTATE")
  {
    return;
  }

  try
  {
    // Parse packet fields
    // Format: BATSTATE:motor_current:nuc_current:wall_voltage:bat_voltage:cell0:cell1:cell2:cell3:cell4:cell_temp:mosfet_temp
    float motor_current = std::stof(packet[1]);
    float nuc_current = std::stof(packet[2]);
    float wall_voltage = std::stof(packet[3]);
    float bat_voltage = std::stof(packet[4]);

    // Parse 5 cell voltages
    std::vector<float> cell_voltages(5);
    for (int i = 0; i < 5; i++)
    {
      cell_voltages[i] = std::stof(packet[5 + i]);
    }

    float cell_temp = std::stof(packet[10]);
    float mosfet_temp = std::stof(packet[11]);

    // Calculate derived values
    float current_sum = motor_current + nuc_current;
    bool present = (bat_voltage > 5.0f);  // Consider battery present if voltage is above 5V threshold

    // Determine power supply status
    uint8_t power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    if (wall_voltage > 5.0f)
    {
      power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
    else if (present)
    {
      power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }

    update(bat_voltage, current_sum, present, cell_voltages, cell_temp, mosfet_temp, power_supply_status);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "BATSTATE error reading packet: %s", e.what());
    return;
  }
}

void PluginBatState::publish()
{
  if (battery_state_pub_)
  {
    battery_state_pub_->publish(*battery_state_msg_);
  }
}

void PluginBatState::update(float voltage, float current, bool present,
                            const std::vector<float>& cell_voltages,
                            float cell_temp, float mosfet_temp,
                            uint8_t power_supply_status)
{
  battery_state_msg_->header.stamp = node_->now();
  battery_state_msg_->voltage = voltage;
  battery_state_msg_->current = current;
  battery_state_msg_->present = present;

  battery_state_msg_->percentage = std::clamp(
      (voltage - BATTERY_VOLTAGE_EMPTY) / (BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY),
      0.0f,
      1.0f);

  // Cell voltages
  battery_state_msg_->cell_voltage.clear();
  for (const auto& cv : cell_voltages)
  {
    battery_state_msg_->cell_voltage.push_back(cv);
  }

  // Temperature (use cell temperature as primary)
  battery_state_msg_->temperature = cell_temp;

  // Store both temperatures in cell_temperature array (cell_temp, mosfet_temp)
  battery_state_msg_->cell_temperature.clear();
  battery_state_msg_->cell_temperature.push_back(cell_temp);
  battery_state_msg_->cell_temperature.push_back(mosfet_temp);

  // Power supply status
  battery_state_msg_->power_supply_status = power_supply_status;
  battery_state_msg_->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

  publish();
}

void PluginBatState::reset()
{
  battery_state_msg_->header.stamp = node_->now();
  battery_state_msg_->voltage = 0.0f;
  battery_state_msg_->current = 0.0f;
  battery_state_msg_->present = false;
  battery_state_msg_->temperature = std::numeric_limits<float>::quiet_NaN();
  battery_state_msg_->cell_voltage.clear();
  battery_state_msg_->cell_temperature.clear();
  battery_state_msg_->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_msg_->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_msg_->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

} // namespace robotont