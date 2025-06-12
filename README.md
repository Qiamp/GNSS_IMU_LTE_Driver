install serial package: sudo apt-get install ros-$ROS_DISTRO-serial
sudo apt install libfmt-dev ros-noetic-paho-mqtt-cpp ros-noetic-paho-mqtt-c
sudo apt-get install libwebsocketpp-dev libjsoncpp-dev
catkin build

连板卡，用Comtool确定串口号：
imu_gnss_driver用STM32那个
gnss_driver用ublox那个
mqtt_all用at那个

启动以上三个的launch文件即可
MQTT监听使用MQTTX

接收端安装编mqtt_client和mqtt2ros_all即可