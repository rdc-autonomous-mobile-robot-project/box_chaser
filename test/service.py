#!/usr/bin/env python3

import rospy
from roscpp.srv import SetLoggerLevel

rospy.wait_for_service('/rosout/set_logger_level')

set_logger_level = rospy.ServiceProxy('/rosout/set_logger_level', SetLoggerLevel)

set_logger_level("rosout","DEBUG")