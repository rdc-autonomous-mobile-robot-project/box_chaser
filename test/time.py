#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool
import time

def main():
    # Initialize the ROS node
    rospy.init_node('your_node_name', anonymous=True)

    start_time = time.time()
    rospy.loginfo("start_time: %s", start_time)
    
    while not rospy.is_shutdown():
        a = time.time() - start_time
        rospy.loginfo("a: %s", a)
        
        if a >= 5.0:
            break  # Exit the loop after 5 seconds

        # You can add your YOLOv5 object detection logic here if needed

        # Sleep for a short duration to control loop rate (optional)
        rospy.sleep(0.1)

    rospy.loginfo("Exiting the loop")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
