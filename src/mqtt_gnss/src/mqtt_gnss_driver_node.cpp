#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>
#include <mqtt_gnss/mqtt_gnss_driver.h>
#include <sstream>
#include <signal.h>  // 用于注册SIGINT信号

// 将输入字符串根据指定分隔符分割为子字符串，返回结果向量
std::vector<std::string> splitString(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// 发送AT指令并打印日志
void sendATCommand(serial::Serial &serial_port, const std::string &command) {
    serial_port.write(command + "\r\n");  // 发送AT指令
    ROS_INFO_STREAM("Sent command: " << command);
}

// 检查串口返回数据中是否含关键字，最多等待3秒
bool checkResponse(serial::Serial &serial_port) {
    ros::Time start = ros::Time::now();
    std::string fullResponse;
    // 循环等待3秒，将所有返回的数据一次性读取
    while ((ros::Time::now() - start) < ros::Duration(3)) {
        size_t avail = serial_port.available();
        if (avail > 0) {
            fullResponse += serial_port.read(avail);
        }
        ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Received response: " << fullResponse);
    // 定义你希望检查的所有关键字
    const std::vector<std::string> keywords = {"OK", "READY", "ok", "ready"};
    // 检查返回数据中是否包含任一关键字
    for (const auto &keyword : keywords) {
        if (fullResponse.find(keyword) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// 辅助函数：检查返回数据中是否包含预期关键字（等待最多2秒）
bool checkResponseFor(serial::Serial &serial_port, const std::string &expected) {
    ros::Time start = ros::Time::now();
    std::string fullResponse;
    while ((ros::Time::now() - start) < ros::Duration(2)) {
        size_t avail = serial_port.available();
        if (avail > 0) {
            fullResponse += serial_port.read(avail);
        }
        ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Expected [" << expected << "], received: " << fullResponse);
    return (fullResponse.find(expected) != std::string::npos);
}

// 全局串口指针，用于SIGINT处理函数中访问串口对象
static serial::Serial* g_serial_ptr = NULL;

// 自定义SIGINT处理函数，在退出前确保断开MQTT连接
void shutdownProcedure(int sig)
{
    if(g_serial_ptr && g_serial_ptr->isOpen())
    {
        ROS_INFO_STREAM("Received SIGINT, initiating shutdown procedure...");
        // 重复发送AT+MDISCONNECT直至接收到OK
        while(ros::ok())
        {
            g_serial_ptr->write("AT+MDISCONNECT\r\n");
            ROS_INFO_STREAM("Sent AT+MDISCONNECT command");
            ros::Time start = ros::Time::now();
            std::string response;
            while ((ros::Time::now() - start) < ros::Duration(2))
            {
                size_t avail = g_serial_ptr->available();
                if(avail > 0)
                {
                    response += g_serial_ptr->read(avail);
                }
                ros::Duration(0.1).sleep();
            }
            ROS_INFO_STREAM("AT+MDISCONNECT response: " << response);
            if(response.find("OK") != std::string::npos)
            {
                break;
            }
            ros::Duration(1).sleep();
        }
        ROS_INFO_STREAM("AT+MDISCONNECT acknowledged, proceeding with AT+MIPCLOSE...");
        // 重复发送AT+MIPCLOSE直至接收到OK
        while(ros::ok())
        {
            g_serial_ptr->write("AT+MIPCLOSE\r\n");
            ROS_INFO_STREAM("Sent AT+MIPCLOSE command");
            ros::Time start = ros::Time::now();
            std::string response;
            while ((ros::Time::now() - start) < ros::Duration(2))
            {
                size_t avail = g_serial_ptr->available();
                if(avail > 0)
                {
                    response += g_serial_ptr->read(avail);
                }
                ros::Duration(0.1).sleep();
            }
            ROS_INFO_STREAM("AT+MIPCLOSE response: " << response);
            if(response.find("OK") != std::string::npos)
            {
                break;
            }
            ros::Duration(1).sleep();
        }
        ROS_INFO_STREAM("MQTT disconnection completed. Shutting down...");
    }
    ros::shutdown();
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg, serial::Serial &serial_port) {
    // 限制数据发送频率，每3秒执行一次
    static ros::Time lastSentTime(0);
    ros::Time now = ros::Time::now();
    if ((now - lastSentTime) < ros::Duration(0.5)) {
        return;
    }
    lastSentTime = now;

    std::string lat_str = std::to_string(msg->latitude);
    std::string lon_str = std::to_string(msg->longitude);
    std::string alt_str = std::to_string(msg->altitude);

    // 发送初始化AT命令，确保网络注册和MQTT配置正常
    std::vector<std::string> pre_commands = {
        "AT+CGREG?",
        "AT+CGATT?",
        "AT+MCONFIG=\"mqttx_769e9e78\",\"test\",\"test\""
    };
    for (const auto &cmd : pre_commands) {
        bool cmd_success = false;
        while (ros::ok() && !cmd_success) {
            sendATCommand(serial_port, cmd);
            cmd_success = checkResponse(serial_port);
            if (!cmd_success) {
                ROS_WARN_STREAM("Command failed: " << cmd << ". Retrying...");
            }
        }
    }
    
    // 启动MQTT连接：等待CONNECT OK消息
    bool mipstart_ok = false;
    while (ros::ok() && !mipstart_ok) {
        sendATCommand(serial_port, "AT+MIPSTART=\"broker.emqx.io\",\"1883\"");
        mipstart_ok = checkResponseFor(serial_port, "CONNECT OK");
        if (!mipstart_ok) {
            ROS_WARN_STREAM("AT+MIPSTART did not return CONNECT OK. Retrying...");
        }
    }
    
    // 建立MQTT会话：等待CONNACK OK消息
    bool mconnect_ok = false;
    while (ros::ok() && !mconnect_ok) {
        sendATCommand(serial_port, "AT+MCONNECT=1,60");
        mconnect_ok = checkResponseFor(serial_port, "CONNACK OK");
        if (!mconnect_ok) {
            ROS_WARN_STREAM("AT+MCONNECT did not return CONNACK OK. Retrying...");
        }
    }
    
    // 持续发布GNSS数据的AT命令
    std::string mpub_cmd = "AT+MPUB=\"test/gnss\",0,0,\"" + lat_str + "," + lon_str + "," + alt_str + "\"";
    while (ros::ok()) {
        sendATCommand(serial_port, mpub_cmd);
        if (checkResponse(serial_port)) {
            ROS_INFO_STREAM("Published GNSS data: " << lat_str << "," << lon_str << "," << alt_str);
        } else {
            ROS_WARN_STREAM("AT+MPUB failed. Retrying...");
        }
        ros::Duration(1).sleep();
    }
}

int main(int argc, char **argv) {
    // 初始化ROS，禁用默认SIGINT处理以注册自定义处理函数
    ros::init(argc, argv, "mqtt_gnss_driver_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("mqtt_gnss_driver_node");

    // 注册自定义SIGINT处理函数，确保断开连接后关闭节点
    signal(SIGINT, shutdownProcedure);

    // 从参数服务器加载串口配置
    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);

    // 初始化并打开串口连接
    serial::Serial serial_port;
    try {
        serial_port.setPort(port);
        serial_port.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port " << port);
        return -1;
    }

    if (!serial_port.isOpen()) {
        ROS_ERROR_STREAM("Failed to open serial port " << port);
        return -1;
    }

    ROS_INFO_STREAM("Serial port opened: " << port << " with baudrate: " << baudrate);

    // 设置全局串口指针，供SIGINT处理函数使用
    g_serial_ptr = &serial_port;

    // 订阅GNSS数据话题
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_driver/receiver_lla", 10, boost::bind(gpsCallback, _1, boost::ref(serial_port)));

    // 进入事件循环
    ros::spin();

    serial_port.close();
    return 0;
}
