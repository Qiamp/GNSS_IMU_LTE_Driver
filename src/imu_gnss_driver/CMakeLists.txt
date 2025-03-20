cmake_minimum_required(VERSION 3.0.2)
project(imu_gnss_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(serial REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

add_executable(imu_gnss_driver_node src/imu_gnss_driver_node.cpp)
target_link_libraries(imu_gnss_driver_node ${catkin_LIBRARIES} ${serial_LIBRARIES})
