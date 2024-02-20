/*
 * This node communicates with Robotont hardware
 */
#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"

int main(int argc, char** argv)
{  
    rclcpp::init(argc, argv);
    auto driver_node = std::make_shared<robotont::Driver>();
    driver_node->initialize();
    
    rclcpp::spin(driver_node);
    rclcpp::shutdown();
}