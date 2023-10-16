#!/usr/bin/env python3

import rospy
from roscpp.srv import GetLoggers

rospy.wait_for_service('/rosout/get_loggers')

get_loggers = rospy.ServiceProxy('/rosout/get_loggers', GetLoggers)

response = get_loggers()

print(response)