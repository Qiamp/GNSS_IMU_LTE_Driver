#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/GNSS_IMU_LTE_Driver/devel/setup.bash

roslaunch imu_gnss_driver imu_gnss_driver.launch & sleep 5
roslaunch ublox_driver ublox_driver.launch & sleep 5
roslaunch mqtt_gnss mqtt_gnss.launch & sleep 5
roslaunch mqtt_imu mqtt_gnss.launch & sleep 5