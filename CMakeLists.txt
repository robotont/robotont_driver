cmake_minimum_required(VERSION 3.5)
project(robotont_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(io_context REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ASIO REQUIRED)
find_package(robotont_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()


find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

include_directories(include)
include_directories(${serial_driver_INCLUDE_DIRS})
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/driver.cpp
  src/hardware.cpp
  src/plugin_base.cpp
  src/driver_node.cpp
  src/plugin_odom.cpp
  src/plugin_motors.cpp
  src/plugin_led_module.cpp
  src/plugin_bat_state.cpp
  src/plugin_joint_state.cpp
)

add_executable(driver_node 
  src/driver.cpp 
  src/hardware.cpp 
  src/driver_node.cpp 
  src/plugin_base.cpp 
  src/plugin_odom.cpp 
  src/plugin_motors.cpp 
  src/plugin_led_module.cpp 
  src/plugin_bat_state.cpp
  src/plugin_joint_state.cpp
)
ament_target_dependencies(driver_node 
  io_context 
  serial_driver 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  pluginlib 
  tf2_geometry_msgs 
  robotont_msgs 
  sensor_msgs
)

add_executable(fake_driver_node 
  src/fake_driver_node.cpp
)
ament_target_dependencies(fake_driver_node 
  io_context 
  serial_driver 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  pluginlib 
  tf2_geometry_msgs
)


install(TARGETS
  driver_node
  fake_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
