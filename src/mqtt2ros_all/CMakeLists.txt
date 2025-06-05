cmake_minimum_required(VERSION 3.0.2)
project(mqtt2ros_all)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mqtt2ros_all src/mqtt2ros_all.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mqtt2ros_all
  ${catkin_LIBRARIES}
)

## Add dependencies to ensure messages are built before the node
add_dependencies(mqtt2ros_all ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})