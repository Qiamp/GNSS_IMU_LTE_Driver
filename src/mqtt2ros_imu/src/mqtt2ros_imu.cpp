#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
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

class Mqtt2RosImu {
public:
    Mqtt2RosImu() {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Subscribe to the /mqtt/imu topic
        imu_sub_ = nh.subscribe("/mqtt/imu", 10, &Mqtt2RosImu::imuCallback, this);

        // Advertise the /parsed_imu topic
        imu_pub_ = nh.advertise<sensor_msgs::Imu>("/mqtt_imu", 10);
    }

private:
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;

    void imuCallback(const std_msgs::String::ConstPtr &msg) {
        // Split the incoming message
        std::vector<std::string> tokens = splitString(msg->data, ',');

        // Ensure the message has the correct number of fields
        if (tokens.size() != 12) {
            ROS_WARN("Received malformed IMU message: %s", msg->data.c_str());
            return;
        }

        try {
            // Parse the fields
            std::string imu_source = tokens[0];
            double sec = std::stod(tokens[1]);
            double nsec = std::stod(tokens[2]);
            double orientation_x = std::stod(tokens[3]);
            double orientation_y = std::stod(tokens[4]);
            double orientation_z = std::stod(tokens[5]);
            double angular_velocity_x = std::stod(tokens[6]);
            double angular_velocity_y = std::stod(tokens[7]);
            double angular_velocity_z = std::stod(tokens[8]);
            double linear_acceleration_x = std::stod(tokens[9]);
            double linear_acceleration_y = std::stod(tokens[10]);
            double linear_acceleration_z = std::stod(tokens[11]);

            // Create and populate the Imu message
            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp.sec = static_cast<uint32_t>(sec);
            imu_msg.header.stamp.nsec = static_cast<uint32_t>(nsec);
            imu_msg.header.frame_id = imu_source;

            imu_msg.orientation.x = orientation_x;
            imu_msg.orientation.y = orientation_y;
            imu_msg.orientation.z = orientation_z;
            imu_msg.orientation.w = 0.0; // Assuming no w component provided

            imu_msg.angular_velocity.x = angular_velocity_x;
            imu_msg.angular_velocity.y = angular_velocity_y;
            imu_msg.angular_velocity.z = angular_velocity_z;

            imu_msg.linear_acceleration.x = linear_acceleration_x;
            imu_msg.linear_acceleration.y = linear_acceleration_y;
            imu_msg.linear_acceleration.z = linear_acceleration_z;

            // Publish the Imu message
            imu_pub_.publish(imu_msg);

        } catch (const std::exception &e) {
            ROS_ERROR("Error parsing IMU message: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mqtt2ros_imu");

    // Create the Mqtt2RosImu object
    Mqtt2RosImu mqtt2ros_imu;

    // Spin to process callbacks
    ros::spin();

    return 0;
}