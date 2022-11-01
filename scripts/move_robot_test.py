#!/usr/bin/env python3
from pdb import post_mortem
from turtle import position
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pynput import keyboard
import sys

KEYBOARD_LISTENER_ENABLED = True


def format_values_to_columns(*args: float, tab_len=10) -> str:
    """
    inputs: numeric arguments
    output: formatted string table look alike
    """
    formatted_string = ""
    for arg in args:
        sign = "-" if arg < 0 else " "
        formatted_string += f"{sign + str(abs(arg)) + ' ' * (tab_len - len(str(abs(arg))))}"

    return formatted_string


class MovementTestNode:
    def __init__(self):
        self.node_name = "movement_test_node"
        rospy.init_node(self.node_name)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, callback=self.odometry_callback)
        self.linear_speed = 0.1
        self.rotation_speed = 1
        self.x = 0
        self.y = 0
        self.z = 0

    def odometry_callback(self, received_message: Odometry):
        rospy.loginfo(self.linear_speed)
        send_command = Twist()
        send_command.linear.x = self.x
        send_command.linear.y = self.y
        send_command.angular.z = self.z
        self.pub.publish(send_command)

        x = round(received_message.twist.twist.linear.x, 4)
        y = round(received_message.twist.twist.linear.y, 4)
        z = round(received_message.twist.twist.angular.z, 4)
        # wheel_speeds_formatted = format_values_to_columns(x, y, z)

        pos_x = round(received_message.pose.pose.position.x, 4)
        pos_y = round(received_message.pose.pose.position.y, 4)
        # position_formatted = format_values_to_columns(pos_x, pos_y)

        orient_z = round(received_message.pose.pose.orientation.z, 4)
        orient_w = round(received_message.pose.pose.orientation.w, 4)
        # orientation_formatted = format_values_to_columns(orient_z, orient_w)
        
        # rospy.loginfo("XYZ SPEEDS: " + format_values_to_columns(self.x, self.y, self.z, x, y, z))
        # rospy.loginfo("ROT SPEEDS: "  + format_values_to_columns(pos_x, pos_y, orient_z))

    def on_press(self, key):
        if key == keyboard.Key.esc:
            sys.exit(0)
        try:
            key = key.char
        except AttributeError:
            key = key.name

        if key == "w":
            self.x = self.linear_speed if self.x == 0 else 0
        elif key == "s":
            self.x = -self.linear_speed if self.x == 0 else 0
        elif key == "a":
            self.y = self.linear_speed if self.y == 0 else 0
        elif key == "d":
            self.y = -self.linear_speed if self.y == 0 else 0
        elif key == "q":
            self.z = self.rotation_speed if self.z == 0 else 0
        elif key == "e":
            self.z = -self.rotation_speed if self.z == 0 else 0
        elif key == "p":
            self.linear_speed += 0.1 if self.linear_speed <= 1 else 1
        elif key == "m":
            self.linear_speed -= 0.1 if self.linear_speed >= 0 else 0
        elif key == "x":
            self.x = 0
            self.y = 0
            self.z = 0

    def start(self):
        if KEYBOARD_LISTENER_ENABLED:
            self.listener = keyboard.Listener(on_press=self.on_press)
            self.listener.start()
            self.listener.join()
        rospy.loginfo(f"{self.node_name} has been started.")
        rospy.spin()


if __name__ == "__main__":
    node = MovementTestNode()
    node.start()
