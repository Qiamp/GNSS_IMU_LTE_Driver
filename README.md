[English](README.md) | [中文](README.zh-CN.md)

# GNSS_IMU_LTE_Driver

A ROS1-based multi-module workspace for integrating IMU, u-blox GNSS, and LTE/MQTT communication, with support for receiver-side replay and web visualization.

The project currently focuses on two typical scenarios:

- Sender side: acquire IMU and GNSS data, then publish it to MQTT through an LTE module driven by serial AT commands.
- Receiver side: subscribe to combined messages from an MQTT broker, reconstruct them as ROS topics, and expose data to a web frontend through WebSocket.

## Project Structure

Core packages:

| Package | Purpose |
| --- | --- |
| `imu_gnss_driver` | Reads IMU data from serial and publishes topics such as `/imu_gnss_driver/imu0/data` and `/imu_gnss_driver/imu1/data` |
| `ublox_driver` | u-blox ZED-F9P driver that publishes PVT, LLA, ephemeris, and related GNSS topics |
| `gnss_comm` | Custom GNSS message definitions and utility functions used by `ublox_driver` |
| `mqtt_all` | Subscribes to IMU + GNSS ROS topics, packs them into one combined message, and forwards it to MQTT through the LTE module |
| `mqtt_gnss` | Publishes GNSS-only data to MQTT |
| `mqtt_imu` | Publishes IMU-only data to MQTT |
| `mqtt_client` | MQTT client bridge that brings broker messages into ROS |
| `mqtt2ros_all` | Parses `/mqtt/combined` back into `/mqtt_imu0`, `/mqtt_imu1`, and `/mqtt_gnss` |
| `visualize_driver` | Subscribes to ROS topics and pushes data to the frontend through WebSocket |

## Data Flow

Typical sender-side flow:

```text
IMU serial device -> imu_gnss_driver -> /imu_gnss_driver/imu*/data
u-blox GNSS -> ublox_driver -> /ublox_driver/receiver_lla, /receiver_pvt ...
ROS topics -> mqtt_all -> LTE module AT commands -> MQTT Broker
```

Typical receiver-side flow:

```text
MQTT Broker -> mqtt_client -> /mqtt/combined
/mqtt/combined -> mqtt2ros_all -> /mqtt_imu0, /mqtt_imu1, /mqtt_gnss
ROS topics -> visualize_driver -> WebSocket(:9002) -> Web page
```

## Requirements

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`

Make sure your ROS environment is initialized correctly and `catkin build` is available before building this workspace.

## Install Dependencies

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

## Build

Run the following in the workspace root:

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
catkin build gnss_comm
catkin build
source devel/setup.bash
```

`gnss_comm` contains custom message definitions. Building it first helps avoid dependency issues during the initial build.

## Configuration Before Launch

### 1. Serial Port Configuration

Update these files based on your actual hardware:

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

Common device names include `/dev/ttyACM0`, `/dev/ttyACM1`, and `/dev/ttyUSB0`.

### 2. MQTT Configuration

Default broker and topic settings:

- Broker: `broker.emqx.io:1883`
- Combined message topic: `test/combined`
- IMU topic: `test/imu`
- GNSS topic: `test/gnss`

Relevant configuration files:

- `src/mqtt_client/config/params.yaml`
- `src/mqtt_all/config/mqtt.yaml`
- `src/mqtt_imu/config/mqtt.yaml`

Note: `mqtt_all.launch` currently loads only the serial configuration file and does not automatically load `mqtt.yaml`. In practice, `mqtt_all` and `mqtt_imu` mainly rely on the default MQTT parameter values in code. If you need to change the broker, credentials, or topics, update the launch file or load the YAML into the parameter server manually.

## Quick Start

### Option 1: Sender Side

Launch in sequence:

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch imu_gnss_driver imu_gnss_driver.launch
roslaunch ublox_driver ublox_driver.launch
roslaunch mqtt_all mqtt_all.launch
```

Or use the helper script:

```bash
bash sender.sh
```

### Option 2: Receiver Side

Launch in sequence:

```bash
cd ~/GNSS_IMU_LTE_Driver
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch mqtt_client standalone.launch
roslaunch mqtt2ros_all all.launch
rosrun visualize_driver visualize_driver
```

Or use:

```bash
bash receiver.sh
```

### Option 3: Single-Machine End-to-End Debugging

To run both sender and receiver chains on the same machine, refer to:

```bash
bash data_collect.sh
```

That script starts:

- `imu_gnss_driver`
- `ublox_driver`
- `mqtt_all`
- `mqtt_client`
- `mqtt2ros_all`

## Key ROS Topics

Raw sender-side data:

- `/imu_gnss_driver/imu0/data`
- `/imu_gnss_driver/imu1/data`
- `/imu_gnss_driver/imu0/mag`
- `/imu_gnss_driver/imu1/mag`
- `/ublox_driver/receiver_pvt`
- `/ublox_driver/receiver_lla`

Recovered data on the MQTT receiver side:

- `/mqtt/combined`
- `/mqtt_imu0`
- `/mqtt_imu1`
- `/mqtt_gnss`

## Visualization

`visualize_driver` starts a WebSocket server by default:

- Address: `ws://<host>:9002`

It currently subscribes to:

- `/imu_gnss_driver/imu0/data`
- `/imu_gnss_driver/imu1/data`
- `/ublox_driver/receiver_lla`

Frontend pages available in the repository:

- `src/visualize_driver/web/index.html`
- `src/Web_Visualize_None_ROS/index.html`

## Helper Scripts

| Script | Purpose |
| --- | --- |
| `sender.sh` | Starts the sender-side chain |
| `receiver.sh` | Starts the receiver-side chain and visualization node |
| `data_collect.sh` | Starts sender and receiver related nodes for single-machine debugging |
| `show.sh` | A simplified startup script similar to `sender.sh` |

## Troubleshooting

### Serial Port Cannot Be Opened

- Check whether the device name is correct.
- Check whether the current user has serial port permissions.
- Use `ls /dev/ttyACM* /dev/ttyUSB*` to verify that the device exists.

### `ublox_driver` Starts but No Data Is Published

- Check `input_serial_port` and baud rate in `src/gnss_driver/config/ipnl_config.yaml`.
- Make sure the receiver is actually outputting UBX data.
- If raw logging is enabled, verify that `to_file` and `dump_dir` are writable.

### No MQTT Messages Are Received

- Check whether the LTE module serial port is configured correctly.
- Check broker address, port, username, and password.
- Use MQTTX or another MQTT client to subscribe to `test/combined` and verify the link.

### Visualization Page Shows No Data

- Make sure `visualize_driver` is running.
- Make sure port `9002` is not occupied.
- Make sure the source ROS topics exist by checking `rostopic list`.

## Notes

- The workspace name is `GNSS_IMU_LTE_Driver`, but the actual ROS package name for the GNSS driver is `ublox_driver`, so use `roslaunch ublox_driver ublox_driver.launch`.
- `mqtt_client` and `mqtt_client_interfaces` come from a standalone MQTT ROS bridge implementation and are included directly in this workspace.
