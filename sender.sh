#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/GNSS_IMU_LTE_Driver/devel/setup.bash

roslaunch imu_gnss_driver imu_gnss_driver.launch & sleep 2
roslaunch ublox_driver ublox_driver.launch & sleep 2
roslaunch mqtt_all mqtt_all.launch & sleep 2
# rosrun visualize_driver visualize_driver