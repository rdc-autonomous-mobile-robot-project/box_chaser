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

        self.flag_desired_distance = True
        self.approached_box = False
        self.detect_box = False
        self.wait_process_start_time = None
        self.wait_process_current_time = 0
        self.wait_process_elapsed_time = 0
        self.time = 0
        self.finish_camera_forward_process_flag = False
        self.camera_send_control_commands_flag = False
        self.back_process_flag = True
        self.camera_send_control_commands_is_finished_flag = True
        self.detected_full_flag = False
        self.blue_box_approached = 0

        self.flag_list = ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]

        self.vel = Twist()
        self.str = String()



        self.detect_box_2 = False

        self.labels = []
        self.filter_strings = ['tag', 'green_box','blue_box']
        self.detected = False
        self.label_string_count = 1
        self.label_msg2 = 0

        self.start_time = None
        self.finish_flag_flag = True

        self.detect_result_client_pro = rospy.ServiceProxy('detect_result', SetBool)
        self.elapsed_time = 0

    #     self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    #     self.detected_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)
    #     self.string_subscriber = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.string_callback)

    #     self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
    #     self.publisher_cmd_vel_by_camera = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.boundingBoxesCallback)
    #     self.calculate_bbox_width = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.calculate_xmax_xmin)
    #     self.laser_scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscanCallback)
    #     self.detect_box_ = rospy.Service("detect_box", SetBool, self.detect_box_srv)
    # #     self.label_string_subscriber = rospy.Subscriber("/label_string", SetBool, self.label_string)

    def detect_result_client(self):
        rospy.wait_for_service('detect_result')
        try:
            service_call = rospy.ServiceProxy('detect_result', SetBool)
            service_call(True)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)



    # Change to accept laser_scan_msg argument
    def loop(self):
        rospy.loginfo("start")
        rospy.loginfo(self.elapsed_time)
        self.detect_result_client()

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()