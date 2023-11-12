#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_srvs.srv import SetBool

class D1_node:
    def __init__(self):
        rospy.init_node('D1_node', anonymous=True)

        self.labels = []
        self.filter_strings = ['tag', 'green_box', 'blue_box']
        self.detected_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)
        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.count_tags_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.count_tags)

        self.tag_a_count = 0
        self.tag_b_count = 0
        self.tag_c_count = 0
        self.max_count = 0

    def count_tags(self, data):
        rospy.loginfo("count_tags started")
        self.labels = [bbox.Class for bbox in data.bounding_boxes]

        for tag in ['tag_a', 'tag_b', 'tag_c']:
            count = self.labels.count(tag)
            setattr(self, f'{tag}_count', count)

        rospy.loginfo("tag_a_count: {}, tag_b_count: {}, tag_c_count: {}".format(
            self.tag_a_count, self.tag_b_count, self.tag_c_count))

        self.max_count = max(self.tag_a_count, self.tag_b_count, self.tag_c_count)
        max_tag = 'tag_c'  # default if counts are the same for all tags

        for tag in ['tag_a', 'tag_b']:
            if getattr(self, f'{tag}_count') == self.max_count:
                max_tag = tag

        self.label_publisher.publish(max_tag)
        rospy.loginfo("Max Count: {}".format(self.max_count))

    def loop(self):
        rospy.loginfo("D1_node loop started")
        rospy.loginfo("Max Count: {}".format(self.max_count))
        # self.count_tags()
        # self.detected_publisher.publish(self.detected)
        # Add other functionalities if needed
        # self.cmd_vel_publisher.publish(some_twist_message)

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()
