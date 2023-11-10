import rospy
from std_msgs.msg import String
import datetime


class CupNoodle(object):
    """
    カップ麺ノード
    """

    def __init__(self):
        """
        ノードの初期化
        """
        rospy.init_node("cup_noodle")

        # 3分周期で実行されるタイマーの定義
        rospy.Timer(rospy.Duration(1), self._timer_callback)

    def _timer_callback(self, event):
        """
        timerコールバック
        """
        # 現在時刻を取得
        now = datetime.datetime.now()
        rospy.loginfo("[{}]3分経過しました。".format(now))

        # システム終了
        rospy.signal_shutdown("3 minutes elapsed")

def main():
    # simple_timerノードの作成
    cn = CupNoodle()
    rospy.loginfo("カップ麺ノードを実行開始します。")

    # 「現在時間」を取得
    now = datetime.datetime.now()
    rospy.loginfo("[{}]カップ麺にお湯を入れました。".format(now))

    try:
        # ノードの実行開始
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("カップ麺ノードを実行終了します.")

if __name__ == "__main__":
    main()
