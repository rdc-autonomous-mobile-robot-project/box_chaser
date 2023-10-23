#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool

rospy.init_node('time')

rospy.loginfo("time1")

rospy.loginfo("time2")

currenttime = rospy.get_rostime()
rospy.loginfo(currenttime)