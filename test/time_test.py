#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class BasicTimerTalker:
    def __init__(self):
        rospy.init_node('basic_timer_talker')
        self.chatter_pub = rospy.Publisher('chatter', String, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        msg = String()
        msg.data = "hello world!"
        rospy.loginfo("publish: %s", msg.data)
        rospy.sleep(3)
        self.chatter_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    talker = BasicTimerTalker()
    talker.run()
