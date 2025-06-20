#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/GNSS_IMU_LTE_Driver/devel/setup.bash

roslaunch mqtt_client standalone.launch & sleep 2
roslaunch mqtt2ros_all all.launch & sleep 2
rosrun visualize_driver visualize_driver