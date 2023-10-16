#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from yolov5_pytorch_ros.msg import BoundingBoxes

# グローバル変数としてwidthを初期化
width = 0.0

def calculate_xmax_xmin(msg):
    global width  # グローバル変数を使用するために必要
    # バウンディングボックスデータを処理
    for bbox in msg.bounding_boxes:
        # バウンディングボックスの xmin と xmax を取得
        xmin = bbox.xmin
        xmax = bbox.xmax

        # 幅を計算
        width = xmax - xmin

def main():
    rospy.init_node('calculate_xmax_xmin')
    
    # Subscribe to the bounding boxes topic
    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, calculate_xmax_xmin)
    
    while not rospy.is_shutdown():
        # widthの値を標準出力に表示
        print(f"バウンディングボックスの幅: {width}")
        rospy.sleep(1.0)  # 表示間隔を調整

if __name__ == '__main__':
    main()
