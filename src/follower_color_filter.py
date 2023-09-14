#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import cv_bridge	#ROSとOpenCV間でデータを受け渡すためのパッケージ

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		cv.namedWindow('window', 1)	#'window'という名前の画像表示のウィンドウを作成
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ

	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')	#画像データをOpenCVに受け渡す
		image = cv.resize(image, (image.shape[1]//2, image.shape[0]//2))	#大きすぎるため，サイズ調整
		cv.imshow('window', image)	#'window'ウィンドウにimageを表示
		cv.waitKey(3)	#3ミリ秒待つ

rospy.init_node('follower')	#'follower'という名前でノードを初期化
follower = Follower()	#Followerクラスのインスタンスを作成（init関数が実行される）
rospy.spin()	#ループ
