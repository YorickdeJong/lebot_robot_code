#!/usr/bin/env python3

import sys
import select
import tty
import termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    """Measures which key strokes are given as input into the terminal and publishes the data on the 
    'keys' topic."""

    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rospy.init_node('keyboard_driver')
    rate = rospy.Rate(100)
    # put command from system into termios -> saves the attributes
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())  # ^^
    print("Publishing keystrokes. Press Crtl-c to exit")

    while not rospy.is_shutdown():

        # instead of blocking stdin -> would not fire any ROS calbacks, we use select()  with a timeout of zero -> returns immidiately
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            key_pub.publish(sys.stdin.read(1))
        rate.sleep()

    # putting constole back to standard mode before we exit the program
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
