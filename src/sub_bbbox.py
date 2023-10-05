#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yolov5_pytorch_ros.msg import BoundingBoxes

position_error = 0.0  # Define position_error as a global variable

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
            send_control_commands()


# Function to send control commands
def send_control_commands():
    global position_error  # Access the global variable position_error
    # Create a Twist message to send control commands
    cmd_vel_msg = Twist()

    # Set the linear and angular velocities
    # Adjust these values as needed
    cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
    cmd_vel_msg.angular.z = -float(position_error)/1000	# Use the value of position_error to set the angular velocity
    # Publish the control commands to the cmd_vel topic
    cmd_vel_publisher.publish(cmd_vel_msg)
    # # Create a Twist message to send control commands
    # cmd_vel_msg = Twist()

    # # Set the linear and angular velocities
    # # Adjust these values as needed
    # cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
    # #cmd_vel_msg.angular.z = 0.5  # Angular velocity (rad/s)

    # # err =  () - 320#黄色の先の重心座標(x)と画像の中心(x)との差
	# # #self.twist.angular.z = -float(position_error)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
    # self.twist.linear.x= -float(position_error)/1000	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
    # # self.cmd_vel_pub.publish(self.twist)

    # # Publish the control commands to the cmd_vel topic
    # cmd_vel_publisher.publish(cmd_vel_msg)

def get_camera_param(msg):
    height = msg.height
    width = msg.width
    print(height, width)
    return height, width


if __name__ == '__main__':
    rospy.init_node('green_box_controller')
    
    # Subscribe to the bounding boxes topic
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)

    get_param_sub = rospy.Subscriber('/usb_cam/image_raw', Image, get_camera_param)
    
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #print(position_error)
    
    rospy.spin()
