#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, SetBoolResponse
from yolov5_pytorch_ros.msg import BoundingBoxes

class RobotController:
    def __init__(self):
        rospy.init_node('D1_node')
        self.position_error = 0.0
        self.width = 0.0
        self.average_range = 0.0
        self.desired_distance = 0.9
        self.start_time = None
        self.flag_desired_distance = True
        self.approached_box = False
        self.detect_box = False
        self.go_on_flag = True
        self.wait_process_start_time = None
        self.wait_process_current_time = 0
        self.wait_process_elapsed_time = 0
        self.time = 0

        rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.boundingBoxesCallback)
        rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, self.calculate_xmax_xmin)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        try:
            detect_box_client = rospy.ServiceProxy('detect_box_2', SetBool)
            response = detect_box_client(True)
            rospy.loginfo(f"Service client response: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def boundingBoxesCallback(self, msg):
        rospy.loginfo("boundingBoxesCallback")
        for box in msg.bounding_boxes:
            if box.Class in ["tag", "green_box", "blue_box", "tag_a", "tag_b", "tag_c"]:
                xmin = box.xmin
                xmax = box.xmax
                self.position_error = (xmin + xmax) / 2.0 - 320  # Adjust 320 as needed
            else:
                rospy.logwarn("No tag or box detected")
            if self.detect_box:
                self.camera_send_control_commands()
            else:
                rospy.loginfo("detect_box is False")

    def camera_send_control_commands(self):
        rospy.loginfo("camera_send_control_commands")
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.1  # Linear velocity (m/s)
        cmd_vel_msg.angular.z = -float(self.position_error) / 1000
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def calculate_xmax_xmin(self, msg):
        for bbox in msg.bounding_boxes:
            xmin = bbox.xmin
            xmax = bbox.xmax
            self.width = xmax - xmin

    def detect_box_srv(self, data):
        rospy.loginfo("detect_box_srv")
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

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            scan_msg = rospy.wait_for_message('/scan', LaserScan)  # LaserScan メッセージを待ち受ける
            self.lidar_send_control_commands(scan_msg)  # 引数に LaserScan メッセージを渡す
            rate.sleep()

    def lidar_send_control_commands(self, msg):
        rospy.loginfo("lidar_send_control_commands")
        num_ranges = len(msg.ranges)  # msg として受け取った LaserScan メッセージを使用
        split_num = int(num_ranges / 2)
        sum_ranges = sum(msg.ranges[split_num - 3:split_num + 4])
        self.average_range = sum_ranges / 7
        rospy.loginfo("Average range: %f", self.average_range)
        if self.average_range > self.desired_distance and self.flag_desired_distance:
            cmd = Twist()
            cmd.linear.x = 0.05
            self.cmd_vel_publisher.publish(cmd)
        elif self.approached_box:
            self.back_process()
        else:
            rospy.loginfo('wait process')
            # self.wait_process_current_time = rospy.get_time()
            # if self.wait_process_start_time is None:
            #     self.wait_process_start_time = self.wait_process_current_time  # 初回のタイムスタンプを記録
            # self.wait_process_elapsed_time = self.wait_process_current_time - self.wait_process_start_time

            # if self.wait_process_start_time is None:
            #     self.wait_process_start_time = rospy.get_time()  # Start the timer when desired_distance is reached
            # else:
            #     rospy.logwarn("start_time is false")
            #     self.wait_process_current_time = rospy.get_time()
            #     self.wait_process_elapsed_time = self.wait_process_current_time - self.wait_process_start_time
            #     rospy.loginfo("elapsed_time: %f", self.wait_process_elapsed_time)

            start_time = time.time()
            while time.time() - start_time < 5.0:

            # if self.wait_process_start_time is None:
            #     self.wait_process_start_time = rospy.get_time()  # Start the timer when desired_distance is reached
            # else:
            #     self.wait_process_current_time = rospy.get_time()
            #     self.wait_process_elapsed_time = self.wait_process_current_time - self.wait_process_start_time
            #     rospy.loginfo("elapsed_time: %f", self.wait_process_elapsed_time)

            # while self.wait_process_elapsed_time < 10.0:
                cmd = Twist()
                cmd.linear.x = 0.000001
                self.cmd_vel_publisher.publish(cmd)
                # rospy.loginfo(self.wait_process_elapsed_time)
                # rospy.loginfo(self.wait_process_current_time)
                # rospy.loginfo(self.start_time)  # <- ここは self.start_time ではなく self.wait_process_start_time
            self.flag_desired_distance = False
            self.approached_box = True

    def back_process(self):
        rospy.loginfo("back process")
        if self.start_time is None:
            self.start_time = rospy.get_time()
        else:
            rospy.logwarn("start_time is false")
        current_time = rospy.get_time()
        elapsed_time = current_time - self.start_time
        cmd = Twist()
        cmd.linear.x = -0.1
        self.cmd_vel_publisher.publish(cmd)
        rospy.loginfo(elapsed_time)
        rospy.loginfo(cmd)
        rospy.loginfo("back process")
        if elapsed_time >= 10.0:
            rospy.loginfo("elapsed_time ok")
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)
            self.finish_flag()
            self.go_on_flag = False
        else:
            rospy.loginfo("elapsed_time is false")

    def finish_flag(self):
        rospy.loginfo("D1_node started")
        try:
            service_call = rospy.ServiceProxy('finish', SetBool)
            service_call(True)
            rospy.loginfo("finish detect_result")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == '__main__':
    controller = RobotController()
    controller.run()
