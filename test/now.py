#!/usr/bin/env python3
import rospy

def now():
    return rospy.get_time()

rospy.init_node("time")

start = now()

while not rospy.is_shutdown():
    # print( now() - start)
    print(now())

