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
        self.time_counter = 0

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
        self.string_subscriber = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.string_callback)

        self.start_time = None
        self.finish_flag_flag = True
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.label_publisher = rospy.Publisher('/label_string', String, queue_size=1)
        self.publisher_cmd_vel_by_camera = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.boundingBoxesCallback)
        self.calculate_bbox_width = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.calculate_xmax_xmin)
        self.laser_scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscanCallback)
        self.detect_box_ = rospy.Service("detect_box", SetBool, self.detect_box_srv)
        self.detect_result_client_func = rospy.ServiceProxy('detect_result', SetBool)


        #count_tags用変数
        self.max_count = 0

        self.count_tags_sub = rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.count_tags)
        self.tag_a_count = 0
        self.tag_b_count = 0
        self.tag_c_count = 0

        self.detect_box_called_count = 0

        self.green_box_process_finished = False
        self.send_detect_result_false_counter =0

        self.green_box_continue_time_counter = 0
        self.blue_box_continue_time_counter = 0

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
            return 'tag_c'  # カウントが全て同じ場合など

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
        # self.label_publisher.publish(self.get_max_tag())
        rospy.loginfo("Max Count: {}".format(self.max_count))



    def string_callback(self, data):
        self.labels = [bbox.Class for bbox in data.bounding_boxes]
        # リストやタプルなどのイテラルオブジェクトの各要素を任意の変数名bboxで取り出してbbox.Classで評価して，その結果を要素とするあらたなリストが返される．
        rospy.loginfo(self.labels)
        if not self.labels:
            rospy.loginfo("self.labels is empty")
            self.detected_full_flag = False
        else:
            rospy.loginfo("self.labels is full")
            self.detected_full_flag = True
        if self.flag_list[0] == self.labels or self.flag_list[1] == self.labels or self.flag_list[2] == self.labels or self.flag_list[3] == self.labels or self.flag_list[4] == self.labels or self.flag_list[5] == self.labels:
            rospy.loginfo("OKKKKKKKKKKKKKKk")
        self.detected = bool(self.labels)
        rospy.loginfo(self.detected)

    def label_string(self):
        self.label_string_count += 1
        rospy.loginfo(self.label_string_count)
        label_str = ' '.join(self.labels)
        label_msg = String()
        label_msg.data = label_str
        self.label_msg2 = label_str
        detection_msg = Bool()
        detection_msg.data = self.detected
        # self.label_publisher.publish(label_msg)
        self.detected_publisher.publish(detection_msg)
        # self.detect_result_client()

    def detect_box_srv(self, data):
        rospy.loginfo("detect_box_srv")
        self.detect_box_called_count += 1
        resp = SetBoolResponse()
        if data.data:
            resp.message = "called"
            resp.success = True
            self.detect_box = True
            self.go_on_flag = True
        else:
            resp.message = "ready"
            resp.success = False
            self.detect_box = False
        return resp

    def finish_flag(self):
        rospy.loginfo("Finish")
        try:
            service_call = rospy.ServiceProxy('finish', SetBool)
            service_call(True)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def boundingBoxesCallback(self, msg):
        for box in msg.bounding_boxes:
            if box.Class in ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]:
                xmin = box.xmin
                xmax = box.xmax
                self.position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed

    def laserscanCallback(self, msg):#現在のプログラムの構成では，正面のrangesしか読んでいないので正面以外から障害物や人が来た時に対応ができない．
        self.laser_scan_msg = msg
        num_ranges = len(self.laser_scan_msg.ranges)
        split_num = int(num_ranges / 2)
        sum_ranges = sum(self.laser_scan_msg.ranges[split_num - 3:split_num + 4])
        self.average_range = sum_ranges / 7

    def calculate_xmax_xmin(self, msg):
        for bbox in msg.bounding_boxes:
            xmin = bbox.xmin
            xmax = bbox.xmax
            self.width = xmax - xmin
            # rospy.loginfo(self.width)

    def camera_send_control_commands(self):#この関数が読み出される時点でx軸方向に前進するため，cmd_velの与え方を変更するべき
        rospy.loginfo("camera_send_control_commands")
        self.vel.linear.x = 0.2  # この行と次の行の動作指令値の与え方を変更する必要がある．具体的には一定以上の値になった時に動作指令値に制限をつける
        self.vel.angular.z = -float(self.position_error) / 1000
        rospy.loginfo(self.width>140)
        self.cmd_vel_publisher.publish(self.vel)

    def lidar_send_control_commands(self):
        rospy.loginfo("average_range: %f", self.average_range)
        rospy.loginfo(self.average_range > self.desired_distance)
        # Check if the average range is greater than the desired distance and the flag_desired_distance is set
        # if self.average_range > self.desired_distance and self.flag_desired_distance:
        if self.average_range > self.desired_distance:
            # Set the linear velocity to move forward (0.05 m/s)
            self.vel.linear.x = 0.2
            # Publish the linear velocity commands
            self.cmd_vel_publisher.publish(self.vel)
            rospy.loginfo("self.vel.linear.x: %f", self.vel.linear.x)
        else:
            # If none of the above conditions are met, wait for a specified time
            rospy.loginfo('wait process')
            start_time = time.time()
            while time.time() - start_time < 6.0:
                # Set a very low linear velocity for waiting
                self.vel.linear.x = 0.000001
                self.vel.angular.z = 0.000001
                # Publish the linear velocity commands for waiting
                self.cmd_vel_publisher.publish(self.vel)
                # Update the flag to indicate that the desired distance has been reached
                self.flag_desired_distance = False
                # Mark that the robot has approached a box
                self.approached_box = True

    def back_process(self):#現在の構成では，規定の時間後退するわけではなく2~3秒後退する状況となっている．
        rospy.loginfo(self.label_msg2)
        rospy.loginfo("back process")

        self.target_time = self.dist / self.speed
        self.vel.linear.x = - self.speed

        # 開始の時刻を保存
        start_time = time.time()
        # 経過した時刻を取得
        end_time = time.time()

        while end_time - start_time <= self.target_time:
            self.cmd_vel_publisher.publish(self.vel)
            end_time = time.time()
        if end_time >= 10.0:#ここのタイミングは要調整
            rospy.loginfo("elapsed_time ok")
            self.go_on_flag = False
            self.back_process_flag = False
            self.approached_box = False
            # self.label_string_count = 1#カウンタを２個設けるためここで1を代入する必要がない
            self.green_box_approached += 1#名前が不適切
        else:
            rospy.loginfo("elapsed_time is false")

    def detect_result_client(self):
        rospy.wait_for_service('detect_result')
        try:
            service_call = rospy.ServiceProxy('detect_result', SetBool)
            service_call(True)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def detect_result_client_false(self):
        rospy.wait_for_service('detect_result')
        try:
            service_call = rospy.ServiceProxy('detect_result', SetBool)
            service_call(False)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)



    # Change to accept laser_scan_msg argument
    def loop(self):
        rospy.loginfo(self.label_string_count)
        rospy.loginfo(self.green_box_approached)
        rospy.loginfo(self.labels)
        rospy.loginfo("self.detect_box_called_count: %f",self.detect_box_called_count)
        rospy.loginfo("D1_node started")

        if self.go_on_flag and self.detect_box and (not self.green_box_process_finished):
            if self.time_counter >= 29 and self.send_detect_result_false_counter < 1:
                self.send_detect_result_false_counter += 1
                self.detect_result_client_false()
            # if self.detected_full_flag and self.green_box_approached < 1 and 'green_box' in self.labels and self.time_counter <= 30 or self.green_box_continue_time_counter <= 10:
            if self.detected_full_flag and self.green_box_approached < 1 and 'green_box' in self.labels or self.time_counter <= 30:
                rospy.loginfo(self.green_box_continue_time_counter)
                self.green_box_continue_time_counter += DURATION
                self.time_counter += DURATION
                rospy.loginfo(self.time_counter)
                self.camera_send_control_commands() # Pass the appropriate laser_scan_msg
                rospy.loginfo("aaaaaaaaaaaaaa")
                # if self.label_string_count < 2 and self.detected:
                rospy.loginfo(self.width)
                rospy.loginfo(self.average_range)
                # if self.width > 140 or (self.width > 30 and self.average_range < 1.0) or self.average_range < 0.45:
                if self.width > 140 or (self.width > 30 and self.average_range < 1.0):
                    if self.label_string_count < 1:#ここは本来二つカウンタを設ける必要がある．
                        self.label_string_publisher(self.get_max_tag())#この行は未検証
                        self.label_string()
                        self.detect_result_client()
                    rospy.loginfo("bbbbbbbbbbbbbbb")
                    self.lidar_send_control_commands()  # Pass the appropriate laser_scan_msg again
                    rospy.loginfo(self.approached_box)
                    if self.approached_box:
                        rospy.loginfo("ccccccccccccc")
                        # self.green_box_approached += 1
                        self.back_process()
                        self.green_box_process_finished = True
        else:
            rospy.loginfo("first step: detect_box is false")

        # if self.go_on_flag and self.detect_box and self.detected_full_flag and self.blue_box_approached == 1:
        # if self.go_on_flag and self.detect_box and self.detected_full_flag and self.label_msg2 == 'blue_box':
        if self.go_on_flag and self.detect_box and self.green_box_process_finished:
            rospy.loginfo(self.blue_box_continue_time_counter)
            if  not ('blue_box' in self.labels):
                self.blue_box_continue_time_counter += DURATION
                rospy.loginfo("blue_box_continue_time_counter: %f",self.blue_box_continue_time_counter)
            elif 'blue_box' in self.labels:
                self.blue_box_continue_time_counter = 0
            if self.detected_full_flag and 'blue_box' in self.labels or self.blue_box_continue_time_counter <= 30:#ここにもカウンタを足す必要がある#detect_full_flagも削る必要がある
                #green_box_detectフラグを条件に変更したほうが良い
                rospy.loginfo("going green_box")
            # Call lidar_send_control_commands with laser_scan_msg argument
                self.camera_send_control_commands() # Pass the appropriate laser_scan_msg
                rospy.loginfo("dddddddddddddd")
                # if self.label_string_count < 2 and self.detected:
                rospy.loginfo(self.width)
                rospy.loginfo(self.average_range)
                # if self.width > 140 or (self.width > 30 and self.average_range < 1.0) or self.average_range < 0.45:
                if self.width > 140 or (self.width > 30 and self.average_range < 1.0):
                    if self.label_string_count < 1:#２つ目のカウンタが必要
                        self.label_string_publisher(self.get_max_tag())#この行は未検証
                        self.label_string()
                        self.detect_result_client()
                    rospy.loginfo("eeeeeeeeeeeeee")
                    self.lidar_send_control_commands()  # Pass the appropriate laser_scan_msg again
                    if self.approached_box:
                        rospy.loginfo("fffffffffffff")
                        self.back_process()
                        # if self.label_msg2 == 'blue_box':
                        if 'blue_box' in self.labels:
                            self.finish_flag()
                            rospy.signal_shutdown('finish')
            elif 'blue_box' not in self.labels:
                rospy.sleep(15.0)
                if self.detect_box_called_count > 2:
                    self.detect_result_client_false()
            # elif self.detect_box_called_count > 2:
            #     self.detect_result_client_false()
            else:
                rospy.loginfo("second step: detect_box is false")
        else:
            rospy.loginfo("second step: detect_box is false")

        rospy.loginfo(self.time_counter)

if __name__ == '__main__':
    D1 = D1_node()
    DURATION = 0.02
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        D1.loop()
        r.sleep()