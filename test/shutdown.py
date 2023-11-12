#!/usr/bin/env python3
import rospy

rospy.init_node('shutdown')
rospy.loginfo('node started')
rospy.sleep(1.0)
rospy.loginfo('node finished')
rospy.signal_shutdown('finish')