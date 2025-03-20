#ifndef MQTT_GNSS_DRIVER_H
#define MQTT_GNSS_DRIVER_H

#include <string>
#include <vector>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>

// 用于分割字符串的辅助函数
std::vector<std::string> splitString(const std::string &s, char delimiter);

// 用于批量发送AT指令的函数
void sendATCommand(serial::Serial &serial_port, const std::string &command);

// 用于检查设备响应（OK 或 ERROR）
bool checkResponse(serial::Serial &serial_port);

#endif // MQTT_GNSS_DRIVER_H
