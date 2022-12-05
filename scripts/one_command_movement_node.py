#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pynput import keyboard
import sys

FILENAME = sys.argv[0].split("/")[-1].replace(".py", "")
KEYBOARD_LISTENER_ENABLED = True


class OneCommandMovementNode:
    KEY_PRESSED = False
    def __init__(self):
        self.node_name = FILENAME
        rospy.init_node(self.node_name)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.sub = rospy.Subscriber("/odom", Odometry, callback=self.odometry_callback)
        self.linear_speed = 0.02
        self.rotation_speed = 0.1
        self.x = 0
        self.y = 0
        self.z = 0
        rospy.loginfo("Default value initialized")
        
        self.start()
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            send_command = Twist()
            send_command.linear.x = self.x
            send_command.linear.y = self.y
            send_command.angular.z = self.z
            
            # if self.KEY_PRESSED:
            self.pub.publish(send_command)
            self.KEY_PRESSED = False
                
            self.x = 0
            self.y = 0
            self.z = 0
            rate.sleep()
        
        

    # def odometry_callback(self, received_message: Odometry):
    #     send_command = Twist()
    #     send_command.linear.x = self.x
    #     send_command.linear.y = self.y
    #     send_command.angular.z = self.z
        
    #     # if self.KEY_PRESSED:
    #     self.pub.publish(send_command)
    #     self.KEY_PRESSED = False
            
    #     self.x = 0
    #     self.y = 0
    #     self.z = 0

    def on_press(self, key):
        rospy.loginfo(key)
        if key == keyboard.Key.esc:
            sys.exit(0)
        try:
            key = key.char
        except AttributeError:
            key = key.name
            
        self.KEY_PRESSED = True

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

    def start(self):
        if KEYBOARD_LISTENER_ENABLED:
            self.listener = keyboard.Listener(on_press=self.on_press)
            self.listener.start()
            # self.listener.join()
        rospy.loginfo(f"Keyboard listener has been started.")
        # rospy.spin()


if __name__ == "__main__":
    node = OneCommandMovementNode()
    node.start()
