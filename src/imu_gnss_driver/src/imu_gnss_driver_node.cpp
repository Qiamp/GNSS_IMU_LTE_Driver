#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#include <string>
#include <vector>
#include "imu_gnss_driver/imu_gnss_driver.h"

// Implement splitString function // 实现 splitString 函数
std::vector<std::string> splitString(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::istringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gnss_driver_node");
    // Use private namespace, load parameters from external config file // 使用私有命名空间，通过外部config文件加载参数
    ros::NodeHandle nh("imu_gnss_driver");

    // Retrieve serial port and baudrate from the parameter server // 从参数服务器获取串口号与波特率
    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);

    // Initialize the serial port // 初始化串口
    serial::Serial serial_port;
    try {
        serial_port.setPort(port);
        serial_port.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open serial port " << port); // 无法打开串口
        return -1;
    }

    if (!serial_port.isOpen()) {
        ROS_ERROR_STREAM("Serial port " << port << " failed to open"); // 串口未成功打开
        return -1;
    }
    ROS_INFO_STREAM("Serial port " << port << " opened successfully, baudrate: " << baudrate); // 串口已打开，波特率

    // Create topic publishers // 建立各主题的发布器
    ros::Publisher imu0_pub = nh.advertise<sensor_msgs::Imu>("imu0/data", 10);
    ros::Publisher imu1_pub = nh.advertise<sensor_msgs::Imu>("imu1/data", 10);
    ros::Publisher mag0_pub = nh.advertise<sensor_msgs::MagneticField>("imu0/mag", 10);
    ros::Publisher mag1_pub = nh.advertise<sensor_msgs::MagneticField>("imu1/mag", 10);
    // ros::Publisher gps_pub  = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        if (serial_port.available()) {
            // Read one line (adjust buffer size and line delimiter as per device requirements) // 读取一行数据（依据设备实际情况，调整缓冲区大小和行结束符）
            std::string line = serial_port.readline(1024, "\n");
            if (line.empty())
                continue;

            // Data format:  
            // imu0_accel_x,imu1_accel_x,imu0_accel_y,imu1_accel_y,imu0_accel_z,imu1_accel_z,
            // imu0_gyro_x,imu1_gyro_x,imu0_gyro_y,imu1_gyro_y,imu0_gyro_z,imu1_gyro_z,
            // imu0_mag_x,imu1_mag_x,imu0_mag_y,imu1_mag_y,imu0_mag_z,imu1_mag_z,longitude,latitude,altitude
            // 数据格式：
            // imu0_accel_x,imu1_accel_x,imu0_accel_y,imu1_accel_y,imu0_accel_z,imu1_accel_z,
            // imu0_gyro_x,imu1_gyro_x,imu0_gyro_y,imu1_gyro_y,imu0_gyro_z,imu1_gyro_z,
            // imu0_mag_x,imu1_mag_x,imu0_mag_y,imu1_mag_y,imu0_mag_z,imu1_mag_z,经度,纬度,高度
            std::vector<std::string> tokens = splitString(line, ',');
            if (tokens.size() < 21) {
                ROS_WARN_STREAM("Incomplete data received: " << line); // 接收到的数据不完整
                continue;
            }

            try {
                double imu0_accel_x = std::stod(tokens[0]);
                double imu1_accel_x = std::stod(tokens[1]);
                double imu0_accel_y = std::stod(tokens[2]);
                double imu1_accel_y = std::stod(tokens[3]);
                double imu0_accel_z = std::stod(tokens[4]);
                double imu1_accel_z = std::stod(tokens[5]);

                double imu0_gyro_x  = std::stod(tokens[6]);
                double imu1_gyro_x  = std::stod(tokens[7]);
                double imu0_gyro_y  = std::stod(tokens[8]);
                double imu1_gyro_y  = std::stod(tokens[9]);
                double imu0_gyro_z  = std::stod(tokens[10]);
                double imu1_gyro_z  = std::stod(tokens[11]);

                double imu0_mag_x   = std::stod(tokens[12]);
                double imu1_mag_x   = std::stod(tokens[13]);
                double imu0_mag_y   = std::stod(tokens[14]);
                double imu1_mag_y   = std::stod(tokens[15]);
                double imu0_mag_z   = std::stod(tokens[16]);
                double imu1_mag_z   = std::stod(tokens[17]);

                // double gps_long = std::stod(tokens[18]);
                // double gps_lat  = std::stod(tokens[19]);
                // double gps_alt  = std::stod(tokens[20]);

                ros::Time current_time = ros::Time::now();

                sensor_msgs::Imu imu0_msg;
                imu0_msg.header.stamp = current_time;
                imu0_msg.header.frame_id = "imu0_link";
                imu0_msg.linear_acceleration.x = imu0_accel_x;
                imu0_msg.linear_acceleration.y = imu0_accel_y;
                imu0_msg.linear_acceleration.z = imu0_accel_z;
                imu0_msg.angular_velocity.x = imu0_gyro_x;
                imu0_msg.angular_velocity.y = imu0_gyro_y;
                imu0_msg.angular_velocity.z = imu0_gyro_z;
                imu0_pub.publish(imu0_msg);

                sensor_msgs::Imu imu1_msg;
                imu1_msg.header.stamp = current_time;
                imu1_msg.header.frame_id = "imu1_link";
                imu1_msg.linear_acceleration.x = imu1_accel_x;
                imu1_msg.linear_acceleration.y = imu1_accel_y;
                imu1_msg.linear_acceleration.z = imu1_accel_z;
                imu1_msg.angular_velocity.x = imu1_gyro_x;
                imu1_msg.angular_velocity.y = imu1_gyro_y;
                imu1_msg.angular_velocity.z = imu1_gyro_z;
                imu1_pub.publish(imu1_msg);

                sensor_msgs::MagneticField mag0_msg;
                mag0_msg.header.stamp = current_time;
                mag0_msg.header.frame_id = "imu0_link";
                mag0_msg.magnetic_field.x = imu0_mag_x;
                mag0_msg.magnetic_field.y = imu0_mag_y;
                mag0_msg.magnetic_field.z = imu0_mag_z;
                mag0_pub.publish(mag0_msg);

                sensor_msgs::MagneticField mag1_msg;
                mag1_msg.header.stamp = current_time;
                mag1_msg.header.frame_id = "imu1_link";
                mag1_msg.magnetic_field.x = imu1_mag_x;
                mag1_msg.magnetic_field.y = imu1_mag_y;
                mag1_msg.magnetic_field.z = imu1_mag_z;
                mag1_pub.publish(mag1_msg);

                // sensor_msgs::NavSatFix gps_msg;
                // gps_msg.header.stamp = current_time;
                // gps_msg.header.frame_id = "gps_link";
                // gps_msg.longitude = gps_long;
                // gps_msg.latitude  = gps_lat;
                // gps_msg.altitude  = gps_alt;
                // gps_pub.publish(gps_msg);

            } catch (std::exception &e) {
                ROS_ERROR_STREAM("Data parsing error: " << e.what()); // 数据解析错误
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    serial_port.close();
    return 0;
}
