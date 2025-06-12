#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <json/json.h>

typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class WebSocketServer {
public:
    WebSocketServer(ros::NodeHandle &nh, int port) 
        : nh_(nh), port_(port) {
        
        // 初始化WebSocket服务器
        server_.init_asio();
        server_.set_open_handler(bind(&WebSocketServer::on_open, this, ::_1));
        server_.set_close_handler(bind(&WebSocketServer::on_close, this, ::_1));
        
        // 订阅ROS话题
        //For Test
        imu0_sub_ = nh_.subscribe("/imu_gnss_driver/imu0/data", 10, &WebSocketServer::imu0Callback, this);
        imu1_sub_ = nh_.subscribe("/imu_gnss_driver/imu1/data", 10, &WebSocketServer::imu1Callback, this);
        //gnss_sub_ = nh_.subscribe("/mqtt_gnss", 10, &WebSocketServer::gnssCallback, this);
    }

    // For Test
    // 发送测试GNSS数据
    void sendTestGnssData() {
        gnssCallback(nullptr);
    }

    void run() {
        server_.listen(port_);
        server_.start_accept();
        server_.run();
    }


private:
    void on_open(websocketpp::connection_hdl hdl) {
        connections_.insert(hdl);
        ROS_INFO("New WebSocket connection");
    }

    void on_close(websocketpp::connection_hdl hdl) {
        connections_.erase(hdl);
        ROS_INFO("WebSocket connection closed");
    }

    void broadcast(const std::string &message) {
        for (auto hdl : connections_) {
            try {
                server_.send(hdl, message, websocketpp::frame::opcode::text);
            } catch (...) {
                ROS_WARN("Failed to send WebSocket message");
            }
        }
    }

    void imu0Callback(const sensor_msgs::Imu::ConstPtr &msg) {
        Json::Value data;
        data["type"] = "imu0";
        data["angular_velocity"]["x"] = msg->angular_velocity.x;
        data["angular_velocity"]["y"] = msg->angular_velocity.y;
        data["angular_velocity"]["z"] = msg->angular_velocity.z;
        data["linear_acceleration"]["x"] = msg->linear_acceleration.x;
        data["linear_acceleration"]["y"] = msg->linear_acceleration.y;
        data["linear_acceleration"]["z"] = msg->linear_acceleration.z;
        broadcast(data.toStyledString());
    }

    void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg) {
        Json::Value data;
        data["type"] = "imu1";
        data["angular_velocity"]["x"] = msg->angular_velocity.x;
        data["angular_velocity"]["y"] = msg->angular_velocity.y;
        data["angular_velocity"]["z"] = msg->angular_velocity.z;
        data["linear_acceleration"]["x"] = msg->linear_acceleration.x;
        data["linear_acceleration"]["y"] = msg->linear_acceleration.y;
        data["linear_acceleration"]["z"] = msg->linear_acceleration.z;
        broadcast(data.toStyledString());
    }

    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        Json::Value data;
        data["type"] = "gnss";
        // 写死的测试值
        data["latitude"] = 22.369606;  // 示例值：纬度
        data["longitude"] = 114.134181;  // 示例值：经度
        data["altitude"] = 30.0;  // 示例值：海拔高度
        // data["latitude"] = msg->latitude;
        // data["longitude"] = msg->longitude;
        // data["altitude"] = msg->altitude;
        broadcast(data.toStyledString());
    }

    ros::NodeHandle nh_;
    server server_;
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections_;
    ros::Subscriber imu0_sub_, imu1_sub_, gnss_sub_;
    int port_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualize_driver");
    ros::NodeHandle nh;
    
    int port = 9002;
    WebSocketServer ws_server(nh, port);

    // 模拟调用 gnssCallback 发送测试数据
    ws_server.sendTestGnssData();
    
    // 在单独线程中运行WebSocket服务器
    std::thread server_thread([&ws_server, port]() {
        ROS_INFO_STREAM("WebSocket server started on port " << port);
        ws_server.run();
    });
    
    ros::spin();
    server_thread.join();
    
    return 0;
}