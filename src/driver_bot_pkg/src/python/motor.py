#!/usr/bin/env python3

import time
import board
from adafruit_motorkit import MotorKit
from geometry_msgs.msg import Twist
import rospy


class Motor:
    def __init__(self):
        """Defines motor throttle, which runs from -1 to 1. Initially the throttle is set to 0. Motors
        1 and 2 are defined to be on the left side, and motors 3 and 4 are defined to be on the right side"""

        self.kit = MotorKit(
            i2c=board.I2C())  # initializes i2c connection to motor
        # drives motor 1 forward (Lf) L = left, R = right, f = front,  r = Rear
        self.kit.motor1.throttle = 0
        self.kit.motor2.throttle = 0  # drives motor 2 forward  (Lr)
        self.kit.motor3.throttle = 0  # drives motor 3 forward (Rf)
        self.kit.motor4.throttle = 0  # drives motor 4 forward (Rr)
        self.xLinearVel = 0
        self.zAngularVel = 0
        self.pubVel = rospy.Publisher('vel', Twist, queue_size=10)

    def initial_val(self):
        """Sets throttle to initial values"""

        self.kit.motor1.throttle = 0.0
        self.kit.motor2.throttle = 0.0
        self.kit.motor3.throttle = 0.0
        self.kit.motor4.throttle = 0.0

    def set_x_linear_vel(self, vel):
        """Sets throttle to be a specified value in the x direction in the robot frame
        Args:
            vel: value between -1 and 1, which sets the throttle. 

        Returns:
            speed in the pos/neg x linear direction in the robot frame. Since wheels turn opposite on left
            and right side, the vel command is set to minus for the right side.
        """
        self.kit.motor1.throttle = vel
        self.kit.motor2.throttle = vel
        self.kit.motor3.throttle = -vel
        self.kit.motor4.throttle = -vel
        self.xLinearVel = (self.kit.motor1.throttle + self.kit.motor2.throttle -
                           self.kit.motor3.throttle - self.kit.motor4.throttle)/4

    def set_z_angular_vel(self, vel):
        """Sets throttle to be a specified value in the z angular direction in the robot frame
        Args:
            vel: value between -1 and 1, which sets the throttle. Pos value equals a right turn

        Returns:
            speed in the pos/neg z angular direction in the robot frame. Wheels turn opposite to eachother, such that
            there is no need to adjust the vel value. 
        """
        self.kit.motor1.throttle = vel
        self.kit.motor2.throttle = vel
        self.kit.motor3.throttle = vel
        self.kit.motor4.throttle = vel
        self.zAngularVel = (self.kit.motor1.throttle + self.kit.motor2.throttle +
                            self.kit.motor3.throttle + self.kit.motor4.throttle)/4
