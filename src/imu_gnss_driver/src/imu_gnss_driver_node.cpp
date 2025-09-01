#include <ros/ros.h>
// 移除serial库，添加Linux原生串口头文件
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>
#include <cstring>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <sstream>
#include <string>
#include <vector>
#include "imu_gnss_driver/imu_gnss_driver.h"
#include <ros/console.h>
#include <map>

// 全局变量存储最新的GPS时间戳
ros::Time latest_gps_timestamp;
bool gps_timestamp_received = false;
// 添加时间偏移变量
ros::Duration time_offset(0.0);
bool time_offset_calculated = false;

// GPS消息回调函数
void gpsCallback(const gnss_comm::GnssPVTSolnMsg::ConstPtr& msg)
{
    // 从GNSS PVT消息中提取时间信息
    // GNSS时间通常包含GPS周数和周内秒数
    uint32_t gps_week = msg->time.week;
    double gps_tow = msg->time.tow; // 周内秒数
    
    // GPS时间起始点：1980年1月6日 00:00:00 UTC
    // 计算GPS时间对应的UTC时间
    const int64_t GPS_EPOCH_UNIX = 315964800; // GPS epoch in Unix time (1980-01-06 00:00:00)
    const int64_t SECONDS_PER_WEEK = 604800;
    
    // 2025年GPS与UTC之间的闰秒差值（GPS时间比UTC快18秒）
    const int GPS_UTC_LEAP_SECONDS = 18;
    
    // 计算UTC时间（加入闰秒修正）
    int64_t utc_seconds = GPS_EPOCH_UNIX + gps_week * SECONDS_PER_WEEK + (int64_t)gps_tow - GPS_UTC_LEAP_SECONDS;
    double fractional_seconds = gps_tow - (int64_t)gps_tow;
    
    // 转换为ROS时间
    latest_gps_timestamp = ros::Time(utc_seconds, fractional_seconds * 1e9);
    gps_timestamp_received = true;
    
    // 计算时间偏移
    ros::Time current_time = ros::Time::now();
    time_offset = latest_gps_timestamp - current_time;
    time_offset_calculated = true;
    
    ROS_DEBUG_STREAM("GPS Week: " << gps_week << ", TOW: " << gps_tow 
                     << ", UTC timestamp: " << latest_gps_timestamp 
                     << ", Time offset: " << time_offset.toSec() << "s");
}

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

// 频率统计相关变量
std::map<std::string, ros::Time> window_start_time;
std::map<std::string, int> message_count_in_window;
std::map<std::string, double> last_calculated_frequency;

// 频率计算函数 - 基于1秒窗口内的消息计数
void calculateAndLogFrequency(const std::string& topic_name) {
    ros::Time now = ros::Time::now();
    
    // 初始化窗口开始时间
    if (window_start_time.find(topic_name) == window_start_time.end()) {
        window_start_time[topic_name] = now;
        message_count_in_window[topic_name] = 0;
    }
    
    // 增加当前窗口内的消息计数
    message_count_in_window[topic_name]++;
    
    // 检查是否已经过了1秒
    ros::Duration elapsed = now - window_start_time[topic_name];
    if (elapsed.toSec() >= 1.0) {
        // 计算频率
        double frequency = message_count_in_window[topic_name] / elapsed.toSec();
        last_calculated_frequency[topic_name] = frequency;
        
        ROS_INFO_STREAM("Topic [" << topic_name << "] frequency: " << frequency << " Hz (messages in " 
                        << elapsed.toSec() << "s: " << message_count_in_window[topic_name] << ")");
        
        // 重置窗口
        window_start_time[topic_name] = now;
        message_count_in_window[topic_name] = 0;
    }
}

// 串口类定义
class SerialPort {
private:
    int fd_;
    std::string port_;
    int baudrate_;
    
    int getBaudrateConstant(int baudrate) {
        switch(baudrate) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default: return B115200;
        }
    }
    
public:
    SerialPort() : fd_(-1) {}
    
    ~SerialPort() {
        close();
    }
    
    bool open(const std::string& port, int baudrate) {
        port_ = port;
        baudrate_ = baudrate;
        
        // 打开串口
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            ROS_ERROR_STREAM("无法打开串口 " << port << ": " << strerror(errno));
            return false;
        }
        
        // 配置串口参数
        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            ROS_ERROR_STREAM("获取串口属性失败: " << strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 设置波特率
        int baud_const = getBaudrateConstant(baudrate);
        cfsetospeed(&tty, baud_const);
        cfsetispeed(&tty, baud_const);
        
        // 8N1配置
        tty.c_cflag &= ~PARENB;        // 无奇偶校验
        tty.c_cflag &= ~CSTOPB;        // 1个停止位
        tty.c_cflag &= ~CSIZE;         // 清除数据位设置
        tty.c_cflag |= CS8;            // 8个数据位
        tty.c_cflag &= ~CRTSCTS;       // 禁用硬件流控制
        tty.c_cflag |= CREAD | CLOCAL; // 启用接收器，忽略调制解调器控制线
        
        // 原始输入模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // 原始输出模式
        tty.c_oflag &= ~OPOST;
        
        // 设置超时
        tty.c_cc[VTIME] = 10;    // 1秒超时
        tty.c_cc[VMIN] = 0;      // 非阻塞读取
        
        // 应用设置
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            ROS_ERROR_STREAM("设置串口属性失败: " << strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 清空缓冲区
        tcflush(fd_, TCIOFLUSH);
        
        return true;
    }
    
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    bool isOpen() const {
        return fd_ >= 0;
    }
    
    bool available() {
        if (fd_ < 0) return false;
        
        fd_set readfds;
        struct timeval timeout;
        
        FD_ZERO(&readfds);
        FD_SET(fd_, &readfds);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000; // 1ms超时
        
        int result = select(fd_ + 1, &readfds, NULL, NULL, &timeout);
        return (result > 0 && FD_ISSET(fd_, &readfds));
    }
    
    std::string readline(size_t max_size = 1024, const std::string& eol = "\r\n") {
        if (fd_ < 0) return "";
        
        std::string line;
        char buffer[1];
        
        while (line.length() < max_size) {
            ssize_t bytes_read = read(fd_, buffer, 1);
            if (bytes_read <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // 非阻塞模式下无数据可读
                    usleep(1000); // 等待1ms
                    continue;
                } else {
                    // 读取错误
                    break;
                }
            }
            
            line += buffer[0];
            
            // 检查行结束符
            if (eol.find(buffer[0]) != std::string::npos) {
                // 找到行结束符，检查是否是完整的结束符
                if (line.length() >= eol.length()) {
                    std::string line_end = line.substr(line.length() - eol.length());
                    if (line_end == eol) {
                        // 移除行结束符并返回
                        line = line.substr(0, line.length() - eol.length());
                        return line;
                    }
                }
            }
        }
        
        return line;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gnss_driver_node");
    // Use private namespace, load parameters from external config file // 使用私有命名空间，通过外部config文件加载参数
    ros::NodeHandle nh("imu_gnss_driver");
    ros::NodeHandle nh_global; // 用于订阅全局话题

    // Retrieve serial port and baudrate from the parameter server // 从参数服务器获取串口号与波特率
    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB0");
    nh.param<int>("baudrate", baudrate, 115200);

    // 使用自定义串口类替换serial::Serial
    SerialPort serial_port;
    if (!serial_port.open(port, baudrate)) {
        ROS_ERROR_STREAM("无法打开串口 " << port);
        return -1;
    }

    if (!serial_port.isOpen()) {
        ROS_ERROR_STREAM("串口 " << port << " 未成功打开");
        return -1;
    }
    ROS_INFO_STREAM("串口 " << port << " 已打开，波特率: " << baudrate);

    // Create topic publishers // 建立各主题的发布器
    ros::Publisher imu0_pub = nh.advertise<sensor_msgs::Imu>("imu0/data", 10);
    ros::Publisher imu1_pub = nh.advertise<sensor_msgs::Imu>("imu1/data", 10);
    ros::Publisher mag0_pub = nh.advertise<sensor_msgs::MagneticField>("imu0/mag", 10);
    ros::Publisher mag1_pub = nh.advertise<sensor_msgs::MagneticField>("imu1/mag", 10);
    // ros::Publisher gps_pub  = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 10);

    // 订阅GPS话题以获取时间戳
    ros::Subscriber gps_sub = nh_global.subscribe("/ublox_driver/receiver_pvt", 10, gpsCallback);

    // 移除固定频率循环，改为依靠串口数据驱动
    //ros::Rate loop_rate(100);
    while (ros::ok())
    {
        // 等待直到有数据可读
        if (serial_port.available()) {
            // 读取一行数据
            std::string line = serial_port.readline(1024, "\r\n");
            if (line.empty()) {
                ros::spinOnce();
                continue;
            }

            // 统计串口接收数据的频率
            calculateAndLogFrequency("serial_data_received");

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
                ros::spinOnce();
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

                //The following code is commented out because the GPS data is not used in this example
                //The GPS Output is relying on the gnss_driver package
                // 由于本示例中未使用 GPS 数据，以下代码被注释掉
                // GPS 输出依赖于 gnss_driver 包
                double gps_long = std::stod(tokens[18]);
                double gps_lat  = std::stod(tokens[19]);
                double gps_alt  = std::stod(tokens[20]);

                ros::Time current_time = ros::Time::now();
                // 计算修正后的时间戳
                ros::Time corrected_timestamp = current_time;
                if (time_offset_calculated) {
                    corrected_timestamp = current_time + time_offset;
                }

                // 为每个传感器创建并发布消息
                sensor_msgs::Imu imu0_msg;
                imu0_msg.header.stamp = corrected_timestamp; // 使用修正后的时间戳
                imu0_msg.header.frame_id = "imu0_link";
                imu0_msg.linear_acceleration.x = imu0_accel_x;
                imu0_msg.linear_acceleration.y = imu0_accel_y;
                imu0_msg.linear_acceleration.z = imu0_accel_z;
                imu0_msg.angular_velocity.x = imu0_gyro_x;
                imu0_msg.angular_velocity.y = imu0_gyro_y;
                imu0_msg.angular_velocity.z = imu0_gyro_z;
                imu0_pub.publish(imu0_msg);
                calculateAndLogFrequency("imu0/data"); // 统计 imu0/data 的发布频率
                // ROS_INFO("Published IMU0 data");  // 添加发布日志

                sensor_msgs::Imu imu1_msg;
                imu1_msg.header.stamp = corrected_timestamp; // 使用修正后的时间戳
                imu1_msg.header.frame_id = "imu1_link";
                imu1_msg.linear_acceleration.x = imu1_accel_x;
                imu1_msg.linear_acceleration.y = imu1_accel_y;
                imu1_msg.linear_acceleration.z = imu1_accel_z;
                imu1_msg.angular_velocity.x = imu1_gyro_x;
                imu1_msg.angular_velocity.y = imu1_gyro_y;
                imu1_msg.angular_velocity.z = imu1_gyro_z;
                imu1_pub.publish(imu1_msg);
                // calculateAndLogFrequency("imu1/data"); // 统计 imu1/data 的发布频率
                // ROS_INFO("Published IMU1 data");  // 添加发布日志

                sensor_msgs::MagneticField mag0_msg;
                mag0_msg.header.stamp = corrected_timestamp; // 使用修正后的时间戳
                mag0_msg.header.frame_id = "imu0_link";
                mag0_msg.magnetic_field.x = imu0_mag_x;
                mag0_msg.magnetic_field.y = imu0_mag_y;
                mag0_msg.magnetic_field.z = imu0_mag_z;
                mag0_pub.publish(mag0_msg);
                // calculateAndLogFrequency("imu0/mag"); // 统计 imu0/mag 的发布频率
                // ROS_INFO("Published MAG0 data");  // 添加发布日志

                sensor_msgs::MagneticField mag1_msg;
                mag1_msg.header.stamp = corrected_timestamp; // 使用修正后的时间戳
                mag1_msg.header.frame_id = "imu1_link";
                mag1_msg.magnetic_field.x = imu1_mag_x;
                mag1_msg.magnetic_field.y = imu1_mag_y;
                mag1_msg.magnetic_field.z = imu1_mag_z;
                mag1_pub.publish(mag1_msg);
                // calculateAndLogFrequency("imu1/mag"); // 统计 imu1/mag 的发布频率
                // ROS_INFO("Published MAG1 data");  // 添加发布日志

                //The following code is commented out because the GPS data is not used in this example
                // 由于本示例中未使用 GPS 数据，以下代码被注释掉
                //The GPS Output is relying on the gnss_driver package
                // GPS 输出依赖于 gnss_driver 包
                // sensor_msgs::NavSatFix gps_msg;
                // gps_msg.header.stamp = current_time; // 使用当前时间戳
                // gps_msg.header.frame_id = "gps_link";
                // gps_msg.longitude = gps_long;
                // gps_msg.latitude  = gps_lat;
                // gps_msg.altitude  = gps_alt;
                // gps_pub.publish(gps_msg);

            } catch (std::exception &e) {
                ROS_ERROR_STREAM("Data parsing error: " << e.what()); //Data parsing error //数据解析错误
            }
        }

        // 保留spinOnce调用
        ros::spinOnce();
    }

    serial_port.close();
    return 0;
}
