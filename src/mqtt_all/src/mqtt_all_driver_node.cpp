#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <mqtt_all/mqtt_all_driver.h>
#include <sstream>
#include <signal.h>  // 用于注册SIGINT信号 //Use to register SIGINT signal

// 将输入字符串根据指定分隔符分割为子字符串，返回结果向量
// Split the input string into substrings based on the specified delimiter and return the result vector
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
// Send AT command and print log
void sendATCommand(serial::Serial &serial_port, const std::string &command) {
    serial_port.write(command + "\r\n");  // 发送AT指令
    ROS_INFO_STREAM("Sent command: " << command);
}

// 通用函数，检查串口返回数据中是否含"OK", "READY", "ok", "ready"等关键字，最多等待3秒
// General function to check if the returned data from the serial port contains keywords such as "OK", "READY", "ok", "ready", etc., waiting for up to 3 seconds at most
bool checkResponse(serial::Serial &serial_port) {
    ros::Time start = ros::Time::now();
    std::string fullResponse;
    // 循环等待3秒，将所有返回的数据一次性读取
    // Loop for 3 seconds to read all the returned data at once
    while ((ros::Time::now() - start) < ros::Duration(0.1)) {
        size_t avail = serial_port.available();
        if (avail > 0) {
            fullResponse += serial_port.read(avail);
        }
        ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Received response: " << fullResponse);
    // 定义你希望检查的所有关键字
    // Define all the keywords you want to check
    const std::vector<std::string> keywords = {"OK", "READY", "ok", "ready"};
    // 检查返回数据中是否包含任一关键字
    // Check if any keyword is contained in the returned data
    for (const auto &keyword : keywords) {
        if (fullResponse.find(keyword) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// 辅助函数：检查返回数据中是否包含预期关键字（自行传入）（等待最多2秒）
// Auxiliary function: check if the returned data contains the expected keyword (passed in by self) (wait for up to 2 seconds at most)
bool checkResponseFor(serial::Serial &serial_port, const std::string &expected) {
    ros::Time start = ros::Time::now();
    std::string fullResponse;
    while ((ros::Time::now() - start) < ros::Duration(0.1)) {
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
// Global serial port pointer for accessing the serial port object in the SIGINT processing function
static serial::Serial* g_serial_ptr = NULL;

// 自定义SIGINT处理函数，在退出前确保断开MQTT连接
// Custom SIGINT processing function to ensure disconnection of MQTT before exiting
void shutdownProcedure(int sig)
{
    if(g_serial_ptr && g_serial_ptr->isOpen())
    {
        ROS_INFO_STREAM("Received SIGINT, initiating shutdown procedure...");
        // 重复发送AT+MDISCONNECT直至接收到OK
        // Repeat sending AT+MDISCONNECT until OK is received
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
        // Repeat sending AT+MIPCLOSE until OK is received
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

// 定义全局变量，用于存储 IMU 和 GNSS 数据
std::string imu0_data, imu1_data, gnss_data;

// IMU0 数据的回调函数
void imu0Callback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu0_data = "imu0," +
                std::to_string(msg->header.stamp.sec) + "," +
                std::to_string(msg->header.stamp.nsec) + "," +
                std::to_string(msg->orientation.x) + "," +
                std::to_string(msg->orientation.y) + "," +
                std::to_string(msg->orientation.z) + "," +
                std::to_string(msg->angular_velocity.x) + "," +
                std::to_string(msg->angular_velocity.y) + "," +
                std::to_string(msg->angular_velocity.z) + "," +
                std::to_string(msg->linear_acceleration.x) + "," +
                std::to_string(msg->linear_acceleration.y) + "," +
                std::to_string(msg->linear_acceleration.z);
}

// IMU1 数据的回调函数
void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg) {
    imu1_data = "imu1," +
                std::to_string(msg->header.stamp.sec) + "," +
                std::to_string(msg->header.stamp.nsec) + "," +
                std::to_string(msg->orientation.x) + "," +
                std::to_string(msg->orientation.y) + "," +
                std::to_string(msg->orientation.z) + "," +
                std::to_string(msg->angular_velocity.x) + "," +
                std::to_string(msg->angular_velocity.y) + "," +
                std::to_string(msg->angular_velocity.z) + "," +
                std::to_string(msg->linear_acceleration.x) + "," +
                std::to_string(msg->linear_acceleration.y) + "," +
                std::to_string(msg->linear_acceleration.z);
}

// 更新 GNSS 数据的回调函数
void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gnss_data = "gnss," +
                std::to_string(msg->latitude) + "," +
                std::to_string(msg->longitude) + "," +
                std::to_string(msg->altitude);
}

// 定时发送综合数据
void publishCombinedData(serial::Serial &serial_port, const std::string &mqtt_topic) {
    // 检查是否所有数据都已更新
    if (imu0_data.empty() || imu1_data.empty() || gnss_data.empty()) {
        ROS_WARN_STREAM("Not all data sources are ready. Skipping publish...");
        return;
    }

    // 构造综合数据字符串
    std::string combined_data = imu0_data + ";" + imu1_data + ";" + gnss_data;

    // 发布综合数据
    std::string mpub_cmd = "AT+MPUB=\"" + mqtt_topic + "\",0,0,\"" + combined_data + "\"";
    sendATCommand(serial_port, mpub_cmd);
    if (checkResponse(serial_port)) {
        ROS_INFO_STREAM("Published combined data to MQTT topic: " << mqtt_topic);
    } else {
        ROS_WARN_STREAM("AT+MPUB failed for combined data. Retrying...");
    }
}

int main(int argc, char **argv) {
    // 初始化ROS，禁用默认SIGINT处理以注册自定义处理函数
    // Initialize ROS, disable default SIGINT handling to register custom handling functions
    ros::init(argc, argv, "mqtt_all_driver_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("mqtt_all_driver_node");

    // 注册自定义SIGINT处理函数，确保断开连接后关闭节点
    // Register custom SIGINT processing function to ensure node shutdown after disconnection
    signal(SIGINT, shutdownProcedure);

    // 从参数服务器加载串口配置
    // Load serial port configuration from the parameter server
    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);

    // 初始化并打开串口连接
    // Initialize and open serial port connection
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
    // Set the global serial port pointer for use by the SIGINT processing function
    g_serial_ptr = &serial_port;

    // MQTT连接配置
    std::string client_id, username, password, broker_host, mqtt_topic;
    int broker_port, clean_session, keepalive;

    nh.param<std::string>("mqtt_config/client_id", client_id, "mqttx_769e9e78");
    nh.param<std::string>("mqtt_config/username", username, "test");
    nh.param<std::string>("mqtt_config/password", password, "test");
    nh.param<std::string>("broker/host", broker_host, "broker.emqx.io");
    nh.param<int>("broker/port", broker_port, 1883);
    nh.param<int>("mconnect/clean_session", clean_session, 1);
    nh.param<int>("mconnect/keepalive", keepalive, 60);
    nh.param<std::string>("topics/combined", mqtt_topic, "test/combined");

    std::vector<std::string> pre_commands = {
        "AT+CGREG?",
        "AT+CGATT?",
        "AT+MCONFIG=\"" + client_id + "\",\"" + username + "\",\"" + password + "\""
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

    // 启动 MQTT 连接
    bool mipstart_ok = false;
    while (ros::ok() && !mipstart_ok) {
        std::string mipstart_cmd = "AT+MIPSTART=\"" + broker_host + "\",\"" + std::to_string(broker_port) + "\"";
        sendATCommand(serial_port, mipstart_cmd);
        mipstart_ok = checkResponseFor(serial_port, "CONNECT OK");
        if (!mipstart_ok) {
            ROS_WARN_STREAM("AT+MIPSTART did not return CONNECT OK. Retrying...");
        }
    }

    // 建立 MQTT 会话
    bool mconnect_ok = false;
    while (ros::ok() && !mconnect_ok) {
        std::string mconnect_cmd = "AT+MCONNECT=" + std::to_string(clean_session) + "," + std::to_string(keepalive);
        sendATCommand(serial_port, mconnect_cmd);
        mconnect_ok = checkResponseFor(serial_port, "CONNACK OK");
        if (!mconnect_ok) {
            ROS_WARN_STREAM("AT+MCONNECT did not return CONNACK OK. Retrying...");
        }
    }

    // 订阅IMU/GNSS数据话题
    // Subscribe to IMU/GNSS data topic
    ros::Subscriber imu0_sub = nh.subscribe<sensor_msgs::Imu>("/imu_gnss_driver/imu0/data", 10, imu0Callback);
    ros::Subscriber imu1_sub = nh.subscribe<sensor_msgs::Imu>("/imu_gnss_driver/imu1/data", 10, imu1Callback);
    ros::Subscriber gnss_sub = nh.subscribe<sensor_msgs::NavSatFix>("/ublox_driver/receiver_lla", 10, gnssCallback);

    // 定时器，用于定时发送综合数据
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), [&serial_port, mqtt_topic](const ros::TimerEvent &) {
        publishCombinedData(serial_port, mqtt_topic);
    });
    
    // 进入事件循环
    // Enter the event loop
    ros::spin();

    serial_port.close();
    return 0;
}
