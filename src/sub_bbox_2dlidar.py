#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yolov5_pytorch_ros.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan

position_error = 0.0  # Define position_error as a global variable
width = 0.0
average_range = 0.0
desired_distance = 0.5

# Callback function for bounding box messages
def boundingBoxesCallback(msg):
    global position_error
    for box in msg.bounding_boxes:
        if box.Class == "tag" or box.Class == "green_box" or box.Class == "blue_box" or box.Class == "tag_a" or box.Class == "tag_b" or box.Class == "tag_c":
            xmin = box.xmin
            xmax = box.xmax
            # Calculate the position error
            position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed
            # Object detected, send control commands
            camera_send_control_commands()

# # Callback function for bounding box messages
# def LaserScanCallback(msg):
#     #rangesの配列の要素数を取得
#     num_ranges = len(msg.ranges)
#     split_num = int(num_ranges/2)
#     sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
#     average_range = sum_ranges / 7
#     print(average_range)
#     if (average_range - 0.3) <= 0.01:
#         lidar_send_control_commands()

# Function to send control commands
def camera_send_control_commands():
    global position_error  # Access the global variable position_error
    # Create a Twist message to send control commands
    cmd_vel_msg = Twist()

    # Set the linear and angular velocities
    # Adjust these values as needed
    cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
    cmd_vel_msg.angular.z = -float(position_error)/1000	# Use the value of position_error to set the angular velocity
    # Publish the control commands to the cmd_vel topic
    # print(average_range)

    # print(average_range)
    if width >= 140:#ここのしきい値は要調整
        rospy.Subscriber('/scan', LaserScan, lidar_send_control_commands)
    else:
        cmd_vel_publisher.publish(cmd_vel_msg)

# # Function to send control commands
def lidar_send_control_commands(msg):
    num_ranges = len(msg.ranges)
    split_num = int(num_ranges / 2)
    sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
    average_range = sum_ranges / 7
    print(average_range)
    if average_range > desired_distance:
        cmd = Twist()
        cmd.linear.x = 0.2  # 0.2 m/sの前進速度（適宜調整）
        cmd_vel_publisher.publish(cmd)
    else:
        cmd = Twist()
        cmd.linear.x = 0.0  # 速度を停止
        cmd_vel_publisher.publish(cmd)

def calculate_xmax_xmin(msg):
    global width  # グローバル変数を使用するために必要
    # バウンディングボックスデータを処理
    for bbox in msg.bounding_boxes:
        # バウンディングボックスの xmin と xmax を取得
        xmin = bbox.xmin
        xmax = bbox.xmax

        # 幅を計算
        width = xmax - xmin
        print(width)

if __name__ == '__main__':
    rospy.init_node('D1_node')
    
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, calculate_xmax_xmin)
    # rospy.Subscriber('/scan', LaserScan, LaserScanCallback)

    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.spin()