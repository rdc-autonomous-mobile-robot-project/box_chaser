#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self, node_name):
        self.node_name = node_name
        self.image_pub = rospy.Publisher(node_name, Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.twist = Twist()	#Twistインスタンス生成

    def callback(self,data):
        try:
            # ROSのsensor_msgs/Image型からOpenCVで処理適量にcv::Mat型へ変換する。
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        h, w = cv_image.shape[:2]
        RESIZE = (w//3, h//3)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([10, 10, 10])
        upper_yellow = np.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)

            err = cx -w// 2#黄色の先の重心座標(x)と画像の中心(x)との差
            self.twist.linear.x = 0.2
			#self.twist.angular.z = -float(err)/100	#画像が大きいためか，-1/100では絶対値がまだ十分に大きく，ロボットが暴れてしまう
            self.twist.angular.z = -float(err)/1000	#誤差にあわせて回転速度を変化させる（-1/1000がP制御でいうところの比例ゲインにあたる）
            self.cmd_vel_pub.publish(self.twist)

        search_top = (h//4)*3
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # ウインドウのサイズを変更
        cv_half_image = cv2.resize(cv_image,   (0,0),fx=0.5, fy=0.5)
        cv_half_masked = cv2.resize(masked, (0,0),fx=0.5, fy=0.5)
        cv_half_mask = cv2.resize(mask, (0,0),fx=0.5, fy=0.5)

        # ウインドウ表示
        cv2.imshow("Origin Image", cv_half_image)
        cv2.imshow("Masked Image", cv_half_masked)
        cv2.imshow("Mask Image", cv_half_mask)
        cv2.waitKey(3)

if __name__ == '__main__':   
    try:
        node_name = 'follower' 
        rospy.init_node(node_name, anonymous=True)
        ImageConverter(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()