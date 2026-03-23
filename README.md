# GNSS_IMU_LTE_Driver

一个基于 ROS1 的多模块驱动工作区，用于接入 IMU、u-blox GNSS 和 LTE/MQTT 通信链路，并支持接收端回放与网页可视化。

项目当前主要覆盖两类场景：

- 发送端：采集 IMU 与 GNSS 数据，通过串口 AT 指令驱动 LTE 模块并发布到 MQTT。
- 接收端：从 MQTT Broker 订阅组合消息，还原为 ROS 话题，并通过 WebSocket 提供前端可视化数据。

## 项目结构

核心包如下：

| 包名 | 作用 |
| --- | --- |
| `imu_gnss_driver` | 从串口读取 IMU 数据，发布 `/imu_gnss_driver/imu0/data`、`/imu_gnss_driver/imu1/data` 等话题 |
| `ublox_driver` | u-blox ZED-F9P 驱动，发布 PVT、LLA、星历等 GNSS 话题 |
| `gnss_comm` | `ublox_driver` 依赖的自定义 GNSS 消息定义与工具函数 |
| `mqtt_all` | 订阅 IMU + GNSS ROS 话题，拼接为一条组合消息，通过 LTE 模块转发到 MQTT |
| `mqtt_gnss` | 仅转发 GNSS 数据到 MQTT |
| `mqtt_imu` | 仅转发 IMU 数据到 MQTT |
| `mqtt_client` | MQTT 客户端，负责从 Broker 桥接消息到 ROS |
| `mqtt2ros_all` | 将 `/mqtt/combined` 组合消息解析回 `/mqtt_imu0`、`/mqtt_imu1`、`/mqtt_gnss` |
| `visualize_driver` | 订阅 ROS 话题并通过 WebSocket 推送给网页前端 |

## 数据链路

典型发送端链路：

```text
IMU 串口设备 -> imu_gnss_driver -> /imu_gnss_driver/imu*/data
u-blox GNSS -> ublox_driver -> /ublox_driver/receiver_lla, /receiver_pvt ...
ROS 话题 -> mqtt_all -> LTE 模块 AT 指令 -> MQTT Broker
```

典型接收端链路：

```text
MQTT Broker -> mqtt_client -> /mqtt/combined
/mqtt/combined -> mqtt2ros_all -> /mqtt_imu0, /mqtt_imu1, /mqtt_gnss
ROS 话题 -> visualize_driver -> WebSocket(:9002) -> Web 页面
```

## 环境要求

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`(Optional, 建议先完成 ROS 基础环境初始化，并确认 `catkin build` 可正常使用。)


## 依赖安装

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-serial \
  ros-noetic-paho-mqtt-cpp \
  ros-noetic-paho-mqtt-c \
  libfmt-dev \
  libwebsocketpp-dev \
  libjsoncpp-dev
```

## 编译

在工作区根目录执行：

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
catkin build gnss_comm
catkin build
source devel/setup.bash
```

`gnss_comm` 包含自定义消息，先单独编译可以减少首次构建时的依赖问题。

## 启动前配置

### 1. 串口配置

根据实际设备修改以下文件：

- `src/imu_gnss_driver/config/serial_params.yaml`
  - `port`
  - `baudrate`
- `src/gnss_driver/config/ipnl_config.yaml`
  - `input_serial_port`
  - `serial_baud_rate`
  - `to_file`
  - `dump_dir`
- `src/mqtt_all/config/serial_params.yaml`
  - `port`
  - `baudrate`
- `src/mqtt_gnss/config/serial_params.yaml`
- `src/mqtt_imu/config/serial_params.yaml`

常见设备名包括 `/dev/ttyACM0`、`/dev/ttyACM1`、`/dev/ttyUSB0`。

### 2. MQTT 配置

默认 Broker 与主题配置如下：

- Broker：`broker.emqx.io:1883`
- 组合消息主题：`test/combined`
- IMU 主题：`test/imu`
- GNSS 主题：`test/gnss`

相关配置文件：

- `src/mqtt_client/config/params.yaml`
- `src/mqtt_all/config/mqtt.yaml`
- `src/mqtt_imu/config/mqtt.yaml`

注意：当前 `mqtt_all.launch` 只加载了串口配置文件，没有自动加载 `mqtt.yaml`。也就是说，`mqtt_all` 和 `mqtt_imu` 代码中的 MQTT 参数主要依赖默认值；如果你要改 Broker、用户名、主题，建议同步修改 launch 文件或手动把 YAML 加载到参数服务器。

## 快速开始

### 方案一：发送端

依次启动：

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch imu_gnss_driver imu_gnss_driver.launch
roslaunch ublox_driver ublox_driver.launch
roslaunch mqtt_all mqtt_all.launch
```

也可以直接执行仓库内脚本：

```bash
bash sender.sh
```

### 方案二：接收端

依次启动：

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch mqtt_client standalone.launch
roslaunch mqtt2ros_all all.launch
rosrun visualize_driver visualize_driver
```

也可以直接执行：

```bash
bash receiver.sh
```

### 方案三：单机联调

如果要在同一台机器上同时跑发送和接收链路，可参考：

```bash
bash data_collect.sh
```

该脚本会顺序启动：

- `imu_gnss_driver`
- `ublox_driver`
- `mqtt_all`
- `mqtt_client`
- `mqtt2ros_all`

## 关键 ROS 话题

发送端原始数据：

- `/imu_gnss_driver/imu0/data`
- `/imu_gnss_driver/imu1/data`
- `/imu_gnss_driver/imu0/mag`
- `/imu_gnss_driver/imu1/mag`
- `/ublox_driver/receiver_pvt`
- `/ublox_driver/receiver_lla`

MQTT 接收端恢复后的数据：

- `/mqtt/combined`
- `/mqtt_imu0`
- `/mqtt_imu1`
- `/mqtt_gnss`

## 可视化

`visualize_driver` 默认启动一个 WebSocket 服务：

- 地址：`ws://<host>:9002`

它当前订阅以下 ROS 话题：

- `/imu_gnss_driver/imu0/data`
- `/imu_gnss_driver/imu1/data`
- `/ublox_driver/receiver_lla`

前端页面可参考：

- `src/visualize_driver/web/index.html`
- `src/Web_Visualize_None_ROS/index.html`

## 常用脚本

| 脚本 | 说明 |
| --- | --- |
| `sender.sh` | 启动发送端链路 |
| `receiver.sh` | 启动接收端链路并打开可视化节点 |
| `data_collect.sh` | 单机联调时同时启动发送与接收相关节点 |
| `show.sh` | 与 `sender.sh` 类似的简化启动脚本 |

## 常见问题

### 串口打不开

- 检查设备名是否正确
- 检查当前用户是否具备串口权限
- 可用 `ls /dev/ttyACM* /dev/ttyUSB*` 确认设备是否存在

### `ublox_driver` 正常启动但没有数据

- 确认 `src/gnss_driver/config/ipnl_config.yaml` 中 `input_serial_port` 与波特率正确
- 确认接收机实际输出 UBX 数据
- 如需保存原始数据，检查 `to_file` 与 `dump_dir` 是否可写

### MQTT 收不到消息

- 检查 LTE 模块串口是否配置正确
- 检查 Broker 地址、端口、用户名和密码
- 使用 MQTTX 等工具订阅 `test/combined` 验证链路

### 可视化页面无数据

- 确认 `visualize_driver` 已启动
- 确认 WebSocket 端口 `9002` 未被占用
- 确认发送端话题确实存在，可通过 `rostopic list` 检查

## 备注

- 顶层工程名为 `GNSS_IMU_LTE_Driver`，但 GNSS 包的 ROS 包名实际是 `ublox_driver`，启动时请使用 `roslaunch ublox_driver ublox_driver.launch`。
- `mqtt_client` 和 `mqtt_client_interfaces` 来自独立 MQTT ROS 桥接实现，当前仓库将其作为工作区一部分直接使用。
