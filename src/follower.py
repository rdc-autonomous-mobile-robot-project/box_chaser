#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

def image_callback(msg):
	#ここに画像データに対する処理を記述する
	pass

rospy.init_node('follower')	#'follower'という名前でノードを初期化
image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)	#Image型で画像トピックを購読し，コールバック関数を呼ぶ
rospy.spin()	#ループ
