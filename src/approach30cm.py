#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarControl:
    def __init__(self):
        rospy.init_node('lidar_control')
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.desired_distance = 0.3  # 目標とする距離 (0.3m)

    def lidar_callback(self, data):
        num_ranges = len(data.ranges)
        split_num = int(num_ranges / 2)
        sum_ranges = sum(data.ranges[split_num - 3:split_num + 4])
        average_range = sum_ranges / 7
        print(average_range)
        if average_range > self.desired_distance:
            cmd = Twist()
            cmd.linear.x = 0.2  # 0.2 m/sの前進速度（適宜調整）
            self.cmd_vel_pub.publish(cmd)
        else:
            cmd = Twist()
            cmd.linear.x = 0.0  # 速度を停止
            self.cmd_vel_pub.publish(cmd)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lidar_control = LidarControl()
    lidar_control.run()
