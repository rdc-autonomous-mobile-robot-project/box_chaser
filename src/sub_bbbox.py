#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from yolov5_pytorch_ros.msg import BoundingBoxes

position_error = 0.0  # Define position_error as a global variable
last_detection_time = None  # Store the time of the last detection

# Callback function for bounding box messages
def boundingBoxesCallback(msg):
    global position_error, last_detection_time
    for box in msg.bounding_boxes:
        if box.Class == "tag" or box.Class == "box":
            xmin = box.xmin
            xmax = box.xmax
            # Calculate the position error
            position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed
            # Object detected, send control commands
            send_control_commands()
            # Update the last detection time
            last_detection_time = rospy.get_time()

# Function to send control commands
def send_control_commands():
    global position_error
    # Create a Twist message to send control commands
    cmd_vel_msg = Twist()

    # Set the linear and angular velocities
    if (rospy.get_time() - last_detection_time) < 5.0:
        # If an object was detected within the last 5 seconds, continue moving
        cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
        cmd_vel_msg.angular.z = -float(position_error) / 1000  # Use the value of position_error to set the angular velocity
    else:
        # If no object was detected for 5 seconds or more, stop moving
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

    # Publish the control commands to the cmd_vel topic
    cmd_vel_publisher.publish(cmd_vel_msg)

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
