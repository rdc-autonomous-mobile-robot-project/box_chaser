#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yolov5_pytorch_ros.msg import BoundingBoxes
from sensor_msgs.msg import LaserScan

position_error = 0.0  # Define position_error as a global variable
width = 0.0
average_range = 0.0
desired_distance = 0.3
start_time = None
flag_desired_distance = True

# Callback function for bounding box messages
def boundingBoxesCallback(msg):
    global position_error, start_time
    for box in msg.bounding_boxes:
        if box.Class == "tag" or box.Class == "green_box" or box.Class == "blue_box" or box.Class == "tag_a" or box.Class == "tag_b" or box.Class == "tag_c":
            xmin = box.xmin
            xmax = box.xmax
            # Calculate the position error
            position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed
            # Object detected, send control commands
            camera_send_control_commands()

# Function to send control commands based on camera data
def camera_send_control_commands():
    global position_error, start_time


    # Create a Twist message to send control commands
    cmd_vel_msg = Twist()

    # Set the linear and angular velocities
    # Adjust these values as needed
    cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
    cmd_vel_msg.angular.z = -float(position_error) / 1000  # Use the value of position_error to set the angular velocity
    # Publish the control commands to the cmd_vel topic

    # Print width for debugging
    print(width)

    if width >= 140:  # ここのしきい値は要調整
        rospy.Subscriber('/scan', LaserScan, lidar_send_control_commands)
    else:
        cmd_vel_publisher.publish(cmd_vel_msg)

# Function to send control commands based on LIDAR data
def lidar_send_control_commands(msg):
    global flag_desired_distance

    num_ranges = len(msg.ranges)
    split_num = int(num_ranges / 2)
    sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
    average_range = sum_ranges / 7
    print(average_range)
    # if average_range > desired_distance and start_time is not None:
    #     current_time = rospy.get_time()
    #     elapsed_time = current_time - start_time
    #     if elapsed_time >= 10.0:
    #         rospy.signal_shutdown("Program finished after 10 seconds")  # プログラムを終了させる

    if average_range > desired_distance and flag_desired_distance:
        cmd = Twist()
        cmd.linear.x = 0.1  # 0.2 m/sの前進速度（適宜調整）
        cmd_vel_publisher.publish(cmd)
    else:
        cmd = Twist()
        cmd.linear.x = 0.0  # 速度を停止
        cmd_vel_publisher.publish(cmd)
        flag_desired_distance = False
        back_process()

def back_process():
    print("neki")
    global start_time
    if start_time is None:
        start_time = rospy.get_time()  # Start the timer when desired_distance is reached
    current_time = rospy.get_time()
    elapsed_time = current_time - start_time
    cmd = Twist()
    cmd.linear.x = -0.1
    cmd_vel_publisher.publish(cmd)
    if elapsed_time >= 10.0:
        cmd.linear.x = 0.0
        cmd_vel_publisher.publish(cmd)

# Callback function for bounding box messages
def calculate_xmax_xmin(msg):
    global width  # グローバル変数を使用するために必要
    # バウンディングボックスデータを処理
    for bbox in msg.bounding_boxes:
        # バウンディングボックスの xmin と xmax を取得
        xmin = bbox.xmin
        xmax = bbox.xmax

        # 幅を計算
        width = xmax - xmin

if __name__ == '__main__':
    rospy.init_node('D1_node')
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, calculate_xmax_xmin)
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while not rospy.is_shutdown():
        # Sleep to reduce CPU usage
        rospy.sleep(0.1)