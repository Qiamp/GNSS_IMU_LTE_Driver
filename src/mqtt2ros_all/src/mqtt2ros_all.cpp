#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#include <vector>

// Helper function to split a string by a delimiter
std::vector<std::string> splitString(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

class Mqtt2RosAll {
public:
    Mqtt2RosAll() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Subscribe to the /mqtt/combined topic
        combined_sub_ = nh.subscribe("/mqtt/combined", 10, &Mqtt2RosAll::combinedCallback, this);

        // Advertise the /mqtt_imu0, /mqtt_imu1, and /mqtt_gnss topics
        imu0_pub_ = nh.advertise<sensor_msgs::Imu>("/mqtt_imu0", 10);
        imu1_pub_ = nh.advertise<sensor_msgs::Imu>("/mqtt_imu1", 10);
        gnss_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/mqtt_gnss", 10);
    }

private:
    ros::Subscriber combined_sub_;
    ros::Publisher imu0_pub_;
    ros::Publisher imu1_pub_;
    ros::Publisher gnss_pub_;

    void combinedCallback(const std_msgs::String::ConstPtr &msg) {
        // Split the incoming message into IMU0, IMU1, and GNSS data
        std::vector<std::string> data_blocks = splitString(msg->data, ';');

        if (data_blocks.size() != 3) {
            ROS_WARN("Received malformed combined message: %s", msg->data.c_str());
            return;
        }

        try {
            // Parse IMU0 data
            std::vector<std::string> imu0_tokens = splitString(data_blocks[0], ',');
            if (imu0_tokens.size() == 12) {
                sensor_msgs::Imu imu0_msg;
                imu0_msg.header.stamp.sec = std::stoi(imu0_tokens[1]);
                imu0_msg.header.stamp.nsec = std::stoi(imu0_tokens[2]);
                imu0_msg.header.frame_id = "imu0";

                imu0_msg.orientation.x = std::stod(imu0_tokens[3]);
                imu0_msg.orientation.y = std::stod(imu0_tokens[4]);
                imu0_msg.orientation.z = std::stod(imu0_tokens[5]);
                imu0_msg.orientation.w = 0.0; // Assuming no w component provided

                imu0_msg.angular_velocity.x = std::stod(imu0_tokens[6]);
                imu0_msg.angular_velocity.y = std::stod(imu0_tokens[7]);
                imu0_msg.angular_velocity.z = std::stod(imu0_tokens[8]);

                imu0_msg.linear_acceleration.x = std::stod(imu0_tokens[9]);
                imu0_msg.linear_acceleration.y = std::stod(imu0_tokens[10]);
                imu0_msg.linear_acceleration.z = std::stod(imu0_tokens[11]);

                imu0_pub_.publish(imu0_msg);
            }

            // Parse IMU1 data
            std::vector<std::string> imu1_tokens = splitString(data_blocks[1], ',');
            if (imu1_tokens.size() == 12) {
                sensor_msgs::Imu imu1_msg;
                imu1_msg.header.stamp.sec = std::stoi(imu1_tokens[1]);
                imu1_msg.header.stamp.nsec = std::stoi(imu1_tokens[2]);
                imu1_msg.header.frame_id = "imu1";

                imu1_msg.orientation.x = std::stod(imu1_tokens[3]);
                imu1_msg.orientation.y = std::stod(imu1_tokens[4]);
                imu1_msg.orientation.z = std::stod(imu1_tokens[5]);
                imu1_msg.orientation.w = 0.0; // Assuming no w component provided

                imu1_msg.angular_velocity.x = std::stod(imu1_tokens[6]);
                imu1_msg.angular_velocity.y = std::stod(imu1_tokens[7]);
                imu1_msg.angular_velocity.z = std::stod(imu1_tokens[8]);

                imu1_msg.linear_acceleration.x = std::stod(imu1_tokens[9]);
                imu1_msg.linear_acceleration.y = std::stod(imu1_tokens[10]);
                imu1_msg.linear_acceleration.z = std::stod(imu1_tokens[11]);

                imu1_pub_.publish(imu1_msg);
            }

            // Parse GNSS data
            std::vector<std::string> gnss_tokens = splitString(data_blocks[2], ',');
            if (gnss_tokens.size() == 4) {
                sensor_msgs::NavSatFix gnss_msg;
                gnss_msg.header.stamp = ros::Time::now();
                gnss_msg.latitude = std::stod(gnss_tokens[1]);
                gnss_msg.longitude = std::stod(gnss_tokens[2]);
                gnss_msg.altitude = std::stod(gnss_tokens[3]);

                gnss_pub_.publish(gnss_msg);
            }

        } catch (const std::exception &e) {
            ROS_ERROR("Error parsing combined message: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mqtt2ros_all");

    // Create the Mqtt2Ros object
    Mqtt2RosAll mqtt2ros_all;

    // Spin to process callbacks
    ros::spin();

    return 0;
}