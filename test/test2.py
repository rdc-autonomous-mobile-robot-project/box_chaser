#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RangeCheckerNode:
    def __init__(self, duration):
        # Initialize the ROS node
        rospy.init_node('range_checker_node')

        # Set the rate at which the node operates based on the provided duration
        self.rate = rospy.Rate(1 / duration)

        # Subscribe to the laser scan topic
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def scan_callback(self, msg):
        # Check if at least one element in the ranges array is less than or equal to 0.2
        if any(distance <= 0.2 for distance in msg.ranges):
            self.publish_cmd_vel()

    def publish_cmd_vel(self):
        # Create a new Twist message
        cmd_vel_msg = Twist()
        
        # Set the linear.x velocity to 0.00001
        cmd_vel_msg.linear.x = 0.00001
        
        # Publish the cmd_vel message
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def loop(self):
        # Perform any other processing if needed
        rospy.loginfo("Executing loop")

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

if __name__ == '__main__':
    # Set the desired duration for rate (e.g., 0.02 seconds)
    DURATION = 0.02

    # Create an instance of the RangeCheckerNode class with the specified duration
    range_checker_node = RangeCheckerNode(DURATION)

    # Run the node
    range_checker_node.run()
