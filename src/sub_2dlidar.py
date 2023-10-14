#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Callback function for bounding box messages
def LaserScanCallback(msg):
    #rangesの配列の要素数を取得
    num_ranges = len(msg.ranges)
    split_num = int(num_ranges/2)
    sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
    average_range = sum_ranges / 7
    if (average_range - 0.3) <= 0.01:
        send_control_commands()

# # Function to send control commands
def send_control_commands():
    print("send_control_commands")
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.0  # Linear velocity (m/s)
    cmd_vel_publisher.publish(cmd_vel_msg)

if __name__ == '__main__':
    rospy.init_node('send_stop_vel')
    
    # Subscribe to the bounding boxes topic
    rospy.Subscriber('/scan', LaserScan, LaserScanCallback)
    
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #print(position_error)
    rospy.spin()