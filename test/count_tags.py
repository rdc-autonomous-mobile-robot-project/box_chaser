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

        #画像用の初期化
        self.position_error = 0.0
        self.width = 0.0
        self.flag_list = ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]

        #レーザスキャン用の初期化
        self.average_range = 0.0
        self.desired_distance = 0.45

        #フラグの初期化
        self.start_time = None
        self.go_on_flag = True

        self.flag_desired_distance = True
        self.approached_box = False
        self.detect_box = False

        self.time = 0
        self.finish_camera_forward_process_flag = False
        self.camera_send_control_commands_flag = False
        self.back_process_flag = True
        self.camera_send_control_commands_is_finished_flag = True
        self.detected_full_flag = False
        self.green_box_approached = 0

        self.vel = Twist()
        self.str = String()

        self.dist = 1.0
        self.speed = 0.2

        self.detect_box_2 = False

        self.labels = []
        self.filter_strings = ['tag', 'green_box','blue_box']
        self.detected = False
        self.label_string_count = 0
        self.label_msg2 = 0
        self.detected_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)
        # self.string_subscriber = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.string_callback)

        self.max_count = 0

        self.start_time = None
        self.finish_flag_flag = True
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.detect_result_client_func = rospy.ServiceProxy('detect_result', SetBool)
        self.count_tags_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.count_tags)

        self.tag_a_count = 0
        self.tag_b_count = 0
        self.tag_c_count = 0



    # def string_callback(self, data):
    #     self.labels = [bbox.Class for bbox in data.bounding_boxes]
        # リストやタプルなどのイテラルオブジェクトの各要素を任意の変数名bboxで取り出してbbox.Classで評価して，その結果を要素とするあらたなリストが返される．
        # rospy.loginfo(self.labels)
        # if not self.labels:
        #     rospy.loginfo("self.labels is empty")
        #     self.detected_full_flag = False
        # else:
        #     rospy.loginfo("self.labels is full")
        #     self.detected_full_flag = True
        # if self.flag_list[0] == self.labels or self.flag_list[1] == self.labels or self.flag_list[2] == self.labels or self.flag_list[3] == self.labels or self.flag_list[4] == self.labels or self.flag_list[5] == self.labels:
        #     rospy.loginfo("OKKKKKKKKKKKKKKk")
        # self.detected = bool(self.labels)
        # rospy.loginfo(self.detected)

    def label_string_publisher(self, max_tag):
        self.label_publisher.publish(max_tag)

    def get_max_tag(self):
        if self.max_count == self.tag_a_count:
            return 'tag_a'
        elif self.max_count == self.tag_b_count:
            return 'tag_b'
        elif self.max_count == self.tag_c_count:
            return 'tag_c'
        else:
            return 'No max tag found'  # カウントが全て同じ場合など

    def count_tags(self, data):
        rospy.loginfo("count_tags started")
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        if 'tag_a' in self.labels:
            self.tag_a_count += 1
        if 'tag_b' in self.labels:
            self.tag_b_count += 1
        if 'tag_c' in self.labels:
            self.tag_c_count += 1
        rospy.loginfo("tag_a_count: {}, tag_b_count: {}, tag_c_count: {}".format(self.tag_a_count, self.tag_b_count, self.tag_c_count))
        self.max_count = max(self.tag_a_count, self.tag_b_count, self.tag_c_count)
        self.label_publisher.publish(self.get_max_tag())
        rospy.loginfo("Max Count: {}".format(self.max_count))



    # Change to accept laser_scan_msg argument
    def loop(self):
        rospy.loginfo("D1_node started")
        self.label_string_publisher(self.get_max_tag())

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()