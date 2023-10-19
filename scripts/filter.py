#!/usr/bin/env python3
import rospy
from yolov5_pytorch_ros.msg import BoundingBoxes
from std_msgs.msg import Header, String, Bool
from rospy.rostime import Time

# Initialize the last detection time
last_detection_time = None

# Initialize the detected flag to False
detected = False

# ラベル情報を格納するリストを初期化
labels = []

# フィルタリングする文字列のリスト
filter_strings = ['tag', 'tag_a', 'tag_b', 'tag_c']

# コールバック関数
def boundingBoxesCallback(data):
    global labels
    global last_detection_time
    global detected

    # ラベル情報を取得
    labels = [bbox.Class for bbox in data.bounding_boxes]

    # 検出が行われたかどうかを設定
    detected = bool(labels)

    # Update the last detection time
    last_detection_time = rospy.get_time()

def now():
    return rospy.get_time()
start = now()

if __name__ == '__main__':
    rospy.init_node('bbox_subscriber')

    # メッセージのパブリッシャーを作成
    label_publisher = rospy.Publisher('/label_string', String, queue_size=10)
    detection_publisher = rospy.Publisher('/detection_status', Bool, queue_size=10)

    rospy.Subscriber('/detected_objects_in_image', BoundingBoxes, boundingBoxesCallback)

    rate = rospy.Rate(10)  # パブリッシュの頻度を設定（10Hzを意味します）

    while not rospy.is_shutdown():
        print(labels)
        while not rospy.is_shutdown():
            print(now() - start)
        
        # Check if more than 10 seconds have passed since the last detection
        start_time = rospy.get_time()
        if start_time > 10:
            detected = False

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
