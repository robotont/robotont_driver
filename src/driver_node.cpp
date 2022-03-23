/*
 * This node communicates with Robotont hardware
 */
#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"

using namespace drivers;
using namespace serial_driver;

int main(int argc, char** argv)
{  

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Driver>();

    //auto node = std::make_shared<Hardware>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
}