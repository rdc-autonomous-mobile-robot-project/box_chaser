#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError	#ROSとOpenCV間でデータを受け渡すためのパッケージ

class Follower:
	def __init__(self):#この関数はバグには関係ないと思われる．最後の行だけ関係あるかもしれない．
		self.bridge = CvBridge()
		cv.namedWindow('window', 1)	#'window'という名前の画像表示のウィンドウを作成
		print('init')#この行のプリント文は標準出力には表示される
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ

	def image_callback(self, msg):
		print('1')

		image = self.bridge.imgmsg_to_cv2(msg, "bgr8")	#画像データをOpenCVに受け渡す
		print('2')
		cv_image = cv.resize(image, (0,0),fx=0.5, fy=0.5)	#大きすぎるため，サイズ調整
		print('3')

		cv.imshow('window_show', cv_image)	#'window'ウィンドウにimageを表示
		print(cv.imshow)
		cv.waitKey(3)	#3ミリ秒待つ

rospy.init_node('follower')	#'follower'という名前でノードを初期化
follower = Follower()	#Followerクラスのインスタンスを作成（init関数が実行される）
rospy.spin()	#ループ