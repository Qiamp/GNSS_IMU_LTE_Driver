#!/usr/bin/env python3

import rosbag
import rospy
import sys
import os
import math
from sensor_msgs.msg import Imu

def convert_deg_to_rad(input_bag_path, output_bag_path):
    """
    将rosbag中IMU陀螺仪数据从度转换为弧度
    
    Args:
        input_bag_path: 输入rosbag文件路径
        output_bag_path: 输出rosbag文件路径
    """
    
    # 检查输入文件是否存在
    if not os.path.exists(input_bag_path):
        print(f"错误: 输入文件不存在: {input_bag_path}")
        return False
    
    # 需要转换的话题列表
    imu_topics = ['/imu_gnss_driver/imu0/data', '/imu_gnss_driver/imu1/data']
    
    # 度到弧度的转换因子
    DEG_TO_RAD = math.pi / 180.0
    
    try:
        with rosbag.Bag(input_bag_path, 'r') as input_bag:
            with rosbag.Bag(output_bag_path, 'w') as output_bag:
                
                # 获取bag文件信息
                info = input_bag.get_type_and_topic_info()
                total_messages = sum([info.topics[topic].message_count for topic in info.topics.keys()])
                processed_messages = 0
                
                print(f"开始处理rosbag文件: {input_bag_path}")
                print(f"总消息数: {total_messages}")
                print(f"需要转换的IMU话题: {imu_topics}")
                
                # 遍历所有消息
                for topic, msg, timestamp in input_bag.read_messages():
                    processed_messages += 1
                    
                    # 显示进度
                    if processed_messages % 1000 == 0:
                        progress = (processed_messages / total_messages) * 100
                        print(f"处理进度: {progress:.1f}% ({processed_messages}/{total_messages})")
                    
                    # 如果是需要转换的IMU话题
                    if topic in imu_topics:
                        # 创建新的IMU消息
                        new_msg = Imu()
                        new_msg.header = msg.header
                        new_msg.orientation = msg.orientation
                        new_msg.orientation_covariance = msg.orientation_covariance
                        new_msg.linear_acceleration = msg.linear_acceleration
                        new_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
                        new_msg.angular_velocity_covariance = msg.angular_velocity_covariance
                        
                        # 转换角速度从度到弧度
                        new_msg.angular_velocity.x = msg.angular_velocity.x * DEG_TO_RAD
                        new_msg.angular_velocity.y = msg.angular_velocity.y * DEG_TO_RAD
                        new_msg.angular_velocity.z = msg.angular_velocity.z * DEG_TO_RAD
                        
                        # 写入转换后的消息
                        output_bag.write(topic, new_msg, timestamp)
                    else:
                        # 其他话题直接复制
                        output_bag.write(topic, msg, timestamp)
                
                print(f"转换完成! 输出文件: {output_bag_path}")
                print(f"处理的消息总数: {processed_messages}")
                
        return True
        
    except Exception as e:
        print(f"处理过程中发生错误: {str(e)}")
        return False

def main():
    """主函数"""
    
    # 检查命令行参数
    if len(sys.argv) != 3:
        print("用法: python3 rosbag_deg2rad.py <输入rosbag文件> <输出rosbag文件>")
        print("示例: python3 rosbag_deg2rad.py input.bag output.bag")
        sys.exit(1)
    
    input_bag_path = sys.argv[1]
    output_bag_path = sys.argv[2]
    
    # 检查输出文件是否已存在
    if os.path.exists(output_bag_path):
        response = input(f"输出文件 {output_bag_path} 已存在，是否覆盖? (y/n): ")
        if response.lower() != 'y':
            print("操作已取消")
            sys.exit(0)
    
    # 执行转换
    success = convert_deg_to_rad(input_bag_path, output_bag_path)
    
    if success:
        print("转换成功完成!")
        sys.exit(0)
    else:
        print("转换失败!")
        sys.exit(1)

if __name__ == "__main__":
    main()
