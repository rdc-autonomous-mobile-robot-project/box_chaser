#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import String, Bool

class Filter:
    def __init__(self):
        rospy.init_node('filter', anonymous=True)
        self.labels = []
        self.filter_strings = ['tag', 'green_box']
        self.detected = False
        self.label_string_count = 1
        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.detected_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)
        self.string_subscriber = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.string_callback)

    def string_callback(self, data):
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        rospy.loginfo(self.labels)
        self.detected = bool(self.labels)
        rospy.loginfo(self.detected)

    def label_string(self):
        self.label_string_count += 1
        rospy.loginfo(self.label_string_count)
        label_str = ' '.join(self.labels)
        label_msg = String()
        label_msg.data = label_str
        detection_msg = Bool()
        detection_msg.data = self.detected
        self.label_publisher.publish(label_msg)
        self.detected_publisher.publish(detection_msg)

if __name__ == '__main__':
    filter_node = Filter()
    DURATION = 0.2
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        if filter_node.label_string_count < 3:  # filter_nodeを使用してクラスの属性にアクセス
            filter_node.label_string()
        r.sleep()