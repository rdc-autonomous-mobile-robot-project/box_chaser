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

if __name__ == '__main__':
    rospy.init_node('green_box_controller')
    
    # Subscribe to the bounding boxes topic
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)
    
    # Create a publisher for the cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rospy.spin()
