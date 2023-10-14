#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Callback function for bounding box messages
def LaserScanCallback(msg):
    #rangesの配列の要素数を取得
    num_ranges = len(msg.ranges)
    split_num = int(num_ranges/2)
    # print(split_num)
    # print(num_ranges)

    sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
    average_range = sum_ranges / 7
    print(sum_ranges)
    print(average_range-0.3)
    if (average_range - 0.3) <= 0.01:
        print("neko")
        send_control_commands()

# # Function to send control commands
def send_control_commands():
    print("send_control_commands")
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.0  # Linear velocity (m/s)
    cmd_vel_publisher.publish(cmd_vel_msg)

def get_lidar_param(msg):
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment
    range_min = msg.range_min
    range_max = msg.range_max
    ranges = msg.ranges
    # print(angle_min, angle_max, angle_increment, range_min, range_max, ranges)
    # print(ranges[0])
    return angle_min, angle_max, angle_increment, range_min, range_max, ranges

if __name__ == '__main__':
    rospy.init_node('send_stop_vel')
    
    # Subscribe to the bounding boxes topic
    rospy.Subscriber('/scan', LaserScan, LaserScanCallback)

    get_param_sub = rospy.Subscriber('/scan', LaserScan, get_lidar_param)#law_scanに変える
    
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #print(position_error)
    
    rospy.spin()