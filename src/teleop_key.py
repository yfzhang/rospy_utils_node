#!/usr/bin/env python

import rospy
import sys
import tty
import select
import termios
from geometry_msgs.msg import Twist
import math
import threading
import time

help = """
Use keyboard as a remote controller
i, k -> x
j, l -> y
w, s -> z
a, d -> yaw
q, e -> pitch
z, c -> roll

CTRL+\ to exit
"""


# TODO: find better ways to detect if a key is being pressed and do it non-blockly. remove the twist reset needs.

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = termios.tcgetattr(sys.stdin)
print(help)
rospy.init_node('teleop_key', log_level=rospy.DEBUG)
twist = Twist()
twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


def update_twist(key):
    if key == 'w':
        twist.linear.z = 1.0
    elif key == 's':
        twist.linear.z = -1.0
    elif key == 'a':
        twist.angular.z = math.radians(10)
    elif key == 'd':
        twist.angular.z = math.radians(-10)
    elif key == 'i':
        twist.linear.x = 1.0
    elif key == 'k':
        twist.linear.x = -1.0
    elif key == 'j':
        twist.linear.y = 1.0
    elif key == 'l':
        twist.linear.y = -1.0
    elif key == 'q':
        twist.angular.y = math.radians(10)
    elif key == 'e':
        twist.angular.y = math.radians(-10)
    elif key == 'z':
        twist.angular.x = math.radians(-10)
    elif key == 'c':
        twist.angular.x = math.radians(10)
    else:
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0


def publish_twist():
    while rospy.is_shutdown() is False:
        twist_pub.publish(twist)
        time.sleep(0.1)


t1 = threading.Thread(target=publish_twist, name='pub_thread')
t1.start()
while rospy.is_shutdown() is False:
    key = get_key()
    if key == '\x03':
        break
    else:
        update_twist(key)
t1.join()
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
