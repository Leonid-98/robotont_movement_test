#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MovementTestNode:
    def __init__(self):
        self.node_name = "movement_test_node"
        rospy.init_node(self.node_name)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, callback=self.odometry_callback)

    def odometry_callback(self, received_message: Odometry):
        send_command = Twist()
        send_command.angular.z = 1.0  # Only want ot rotate robot for now
        rospy.loginfo(received_message)

    def start(self):
        rospy.loginfo(f"{self.node_name} has been started.")
        rospy.spin()


if __name__ == "__main__":
    node = MovementTestNode()
    node.start()
