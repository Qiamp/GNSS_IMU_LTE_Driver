# ROS 软件包安装与使用指南

## 环境要求
- ROS Noetic (或其他对应 `$ROS_DISTRO` 版本)
- Ubuntu 系统

## 安装步骤

### 1. 安装依赖包
执行以下命令安装所需依赖：

```bash
# 串口通信支持
sudo apt-get install ros-$ROS_DISTRO-serial

# MQTT 和格式化库
sudo apt install libfmt-dev ros-noetic-paho-mqtt-cpp ros-noetic-paho-mqtt-c

# WebSocket 和 JSON 库
sudo apt-get install libwebsocketpp-dev libjsoncpp-dev

# 编译工作空间
#先编译gnss_comm
catkin build gnss_comm
#完整编译
catkin build
```

### 2. 硬件连接配置
- 连接硬件板卡后，使用 `Comtool` 工具确定串口号
- 驱动对应关系：
  - `imu_gnss_driver`：用于 STM32 设备
  - `gnss_driver`：用于 u-blox GNSS 模块
  - `mqtt_all`：用于 AT 指令通信

### 3. 启动节点
使用对应的 launch 文件启动节点：
```bash
cd ~/GNSS_IMU_LTE_Driver
source ~/GNSS_IMU_LTE_Driver/devel/setup.bash
roslaunch imu_gnss_driver imu_gnss_driver.launch
roslaunch ublox_driver ublox_driver.launch
```

### 4. MQTT 监控
使用 [MQTTX](https://mqttx.app/) 工具监听 MQTT 消息

## 接收端配置
在接收端仅需要安装编译以下包：
- `mqtt_client`
- `mqtt2ros_all`

## 常见问题
- 确保串口设备有正确权限（如 `/dev/ttyACM*`）
- 检查 ROS 环境变量是否生效（`source devel/setup.bash`）
- 首次运行前建议执行 `catkin clean` 后重新编译