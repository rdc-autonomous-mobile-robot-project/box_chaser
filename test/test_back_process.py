#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool

class D1_node:
    def __init__(self):
        rospy.init_node('D1_node', anonymous=True)
        self.position_error = 0.0
        self.width = 0.0
        self.average_range = 0.0
        self.desired_distance = 0.45
        self.start_time = None
        self.go_on_flag = True

        self.approached_box = False
        self.time = 0
        self.back_process_flag = True
        self.camera_send_control_commands_is_finished_flag = True
        self.detected_full_flag = False
        self.blue_box_approached = 0

        self.vel = Twist()
        self.str = String()

        self.label_string_count = 1

        self.start_time = None
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


        self.dist = 1.0
        self.speed = 0.2

    def back_process(self):#現在の構成では，規定の時間後退するわけではなく2~3秒後退する状況となっている．
        rospy.loginfo("back process")
        # rospy.loginfo("self.average_range: %f", self.average_range)
        if self.start_time is None:
            self.start_time = rospy.get_time()
        else:
            rospy.logwarn("start_time is false")
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time
        self.vel.linear.x = -0.3
        self.cmd_vel_publisher.publish(self.vel)
        rospy.loginfo(elapsed_time)
        rospy.loginfo("back process")
        if elapsed_time >= 10.0:
            rospy.loginfo("elapsed_time ok")
            self.go_on_flag = False
            self.back_process_flag = False
            self.approached_box = False
            self.label_string_count = 1
            self.blue_box_approached += 1
        else:
            rospy.loginfo("elapsed_time is false")

    # Change to accept laser_scan_msg argument
    def loop(self):
        rospy.loginfo("D1_node started")
        self.back_process()

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()