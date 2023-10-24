#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool

# デフォルトで検出が行われていないと仮定
detected = False

# ラベル情報を格納するリストを初期化
labels = []

# フィルタリングする文字列のリスト
filter_strings = ['tag','green_box','blue_box']

# コールバック関数
def boundingBoxesCallback(data):
    global detected  # グローバル変数を変更するために必要
    global labels

    # ラベル情報を取得
    labels = [bbox.Class for bbox in data.bounding_boxes]

    # 検出が行われたかどうかを設定
    detected = bool(labels)

if __name__ == '__main__':
    rospy.init_node('bbox_subscriber')

    # メッセージのパブリッシャーを作成
    label_publisher = rospy.Publisher('/label_string', String, queue_size=10)
    detection_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)

    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)

    rate = rospy.Rate(10)  # パブリッシュの頻度を設定（10Hzを意味します）

    while not rospy.is_shutdown():
        print(labels)
        
        # フィルタリング
        filtered_labels = list(filter(lambda label: label not in filter_strings, labels))
        
        # ラベル情報を結合してスペースで区切った文字列に変換
        label_str = ' '.join(filtered_labels)
        
        # 新しいメッセージを作成
        label_msg = String()
        label_msg.data = label_str

        # 検出ステータスを新しいメッセージに設定
        detection_msg = Bool()
        detection_msg.data = detected

        # ラベル情報と検出ステータスをパブリッシュ
        label_publisher.publish(label_msg)
        detection_publisher.publish(detection_msg)

        rate.sleep()
