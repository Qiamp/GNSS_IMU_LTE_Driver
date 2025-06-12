cmake_minimum_required(VERSION 3.0.2)
project(visualize_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Threads REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/jsoncpp
)

catkin_package()

add_executable(visualize_driver src/visualize_driver.cpp)
target_link_libraries(visualize_driver
  ${catkin_LIBRARIES}
  jsoncpp
  ${CMAKE_THREAD_LIBS_INIT}
)

install(TARGETS visualize_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)