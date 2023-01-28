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
)

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
  src/plugin_odom_expected.cpp
  src/plugin_debug_out.cpp
) 

add_executable (fake_driver_node src/fake_driver_node.cpp)

target_link_libraries(fake_driver_node ${catkin_LIBRARIES})
target_link_libraries(driver_node ${catkin_LIBRARIES})



install(TARGETS driver_node fake_driver_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
