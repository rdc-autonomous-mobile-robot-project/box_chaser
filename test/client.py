#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool

if __name__ == "__main__":
    try:
        detect_box_client = rospy.ServiceProxy('detect_box_2', SetBool)
        response = detect_box_client(True)  # Send a request to start detection
        rospy.loginfo(f"Service client response: {response.message}")
    except rospy.ServiceException as e:        rospy.logerr(f"Service call failed: {e}")