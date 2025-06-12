#!/bin/bash
source /opt/ros/noetic/setup.bash
source ~/GNSS_IMU_LTE_Driver/devel/setup.bash

roslaunch imu_gnss_driver imu_gnss_driver.launch & sleep 5
rosrun visualize_driver visualize_driver