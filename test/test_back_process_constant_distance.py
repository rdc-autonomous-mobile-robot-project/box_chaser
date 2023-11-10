#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist

class Test():
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.dist = 1.0
        self.speed = 0.2
        self.vel = Twist()

    # 直進
    def pub_x(self):
        self.target_time = self.dist / self.speed
        self.vel.linear.x = - self.speed

        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()
        rospy.loginfo("start_time: %s", start_time)
        rospy.loginfo("end_time: %s", end_time) 
        rospy.loginfo("difference_time: %s", start_time - end_time)
        while end_time - start_time <= self.target_time:
            self.pub.publish(self.vel)
            end_time = time.time()

    def loop(self):
        self.pub_x()

if __name__ == '__main__':
    rospy.init_node('tcmdvel_publisher')
    DURATION = 0.02
    test = Test()
    r = rospy.Rate(1/DURATION)
    while not rospy.is_shutdown():
        test.loop()
        r.sleep()