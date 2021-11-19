cmake_minimum_required(VERSION 2.8.3)
project(robotont_driver)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  robotont_msgs
  tf
  serial
  class_loader
)

# class_loader is included as a non-ROS library for ROS2 compatibility
find_package(class_loader REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    robotont_msgs
    tf
    serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${class_loader_INCLUDE_DIRS}
)

add_executable(driver_node 
  src/driver_node.cpp
  src/driver.cpp
  src/hardware.cpp 
  src/plugin_base.cpp 
  src/plugin_motors.cpp 
  src/plugin_odom.cpp
  src/plugin_power_supply.cpp
  src/plugin_range.cpp
  src/plugin_led_module.cpp
) 

add_executable (fake_driver_node src/fake_driver_node.cpp)
add_executable (plugin_yaml_parser src/examples/plugin_yaml_parser.cpp)
add_executable (plugin_loader src/examples/plugin_loader.cpp)

target_link_libraries(fake_driver_node ${catkin_LIBRARIES})
target_link_libraries(driver_node ${catkin_LIBRARIES})
target_link_libraries(plugin_yaml_parser ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(plugin_loader ${catkin_LIBRARIES} ${class_loader_LIBRARIES})

# Build the plugins
add_library(plugin_test_1 src/examples/plugin_test_1.cpp)
target_link_libraries(plugin_test_1 ${catkin_LIBRARIES} ${class_loader_LIBRARIES})
class_loader_hide_library_symbols(plugin_test_1)

install(TARGETS driver_node fake_driver_node plugin_test_1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
