cmake_minimum_required(VERSION 3.0.2)
project(mqtt_gnss)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

add_executable(mqtt_gnss_driver_node src/mqtt_gnss_driver_node.cpp)
target_link_libraries(mqtt_gnss_driver_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
