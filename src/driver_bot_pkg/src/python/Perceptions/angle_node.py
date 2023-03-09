#!/usr/bin/env python3

# 3rd packages
import cv2
import rospy
import numpy as np

# 3rd party messages
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# own messages
from driver_bot_pkg.msg import ObjectAngle
from driver_bot_pkg.msg import LidarData


class AngleDetection():
    # ---------overriding----------#
    def __init__(self):

        self.pubGamma = rospy.Publisher('gamma', Float64, queue_size=10)
        self.pubObjectAngle = rospy.Publisher(
            'objectPosition', ObjectAngle, queue_size=10)
        self.subLidar = rospy.Subscriber(
            'rplidar', LidarData, self.lidarCallback)

        self.dist = 10
        self.angle_increment = 1
        self.angle = 1

    def lidarCallback(self, msg):
        """Callback function containing data from rplidar
        Args:
            msg: contains data for distance and angle_increment
        """
        self.angle_increment = msg.angle_increment
        self.dist = msg.distance

    def dataAnalyses(self):
        """Calculates the angle at which the minimum distance to an object occurs"""
        unit_circle = 2*np.pi
        increments_in_circle = unit_circle/self.angle_increment
        angle = self.minimumDistanceArg()/increments_in_circle * 2
        # print('angle is: ' + str(angle) + " pi")

        self.angle = angle
        print(self.angle)

    def detection(self):
        """Check where the wall of the robot is and adjust the angle and
        make sure that new angle is always smaller than 1/2 for gamma calc
        Args:
            angle: angle at which minimum distance occurs

        Returns:
            rightSide: True if wall is on the right side of the vehicle
            angle_new: adjusted angle for position of the wall compared to the vehicle
            sign: contains direction of the vehicle's y velocity in world coordinates
            publishes data on 'objectPosition' topic
        """
        self.dataAnalyses()
        if 1/5 < self.angle < 1/2:
            # moving towards left wall
            rightSide = False
            angle_new = self.angle
            sign = -1

        if 1/2 < self.angle < 1:
            # moving away from left wall
            rightSide = False
            angle_new = self.angle - 1/2
            sign = 1

        if 1 < self.angle < 3/2:
            # moving away from right wall
            angle_new = self.angle - 1
            rightSide = True
            sign = -1

        if 3/2 < self.angle < 9/5:
            # moving towards right wall
            angle_new = self.angle - 3/2
            rightSide = True
            sign = 1

        if self.angle <= 1/5 or self.angle >= 9/5:
            # Will not get triggered by wall
            # Prevents getting stuck
            rightSide = False
            angle_new = 0
            sign = 0

        return rightSide, angle_new, sign

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rightSide, angle_new, sign = self.detection()
            print('angle: ' + str(self.angle))
            self.pubGammaData(angle_new)
            self.pubObjectLocation(rightSide, angle_new, sign)

            rate.sleep()
    # ---------additional functions----------#

    def pubGammaData(self, angle_new):
        """publishes angle between robot frame and world frame"""
        self.pubGamma.publish(1/2 - angle_new)

    def pubObjectLocation(self, rightSide, angle_new, sign):
        """publishes data about orientation of vehicle compared to object
        Args:
            rightSide: true if object is on the right side of the vehicle

            angle_new: adjusted angle based on in which quadrant the object is located in the robot frame

            sign: 1 if y velocity is positive, -1 if negative
        """
        msg = ObjectAngle()
        msg.rightSide = rightSide
        msg.angle_new = angle_new
        msg.sign = sign

        self.pubObjectAngle.publish(msg)

    # ---------support functions----------#

    def minimumDistanceArg(self):
        """Calculates index for a minimum distance"""
        return np.argmin(self.dist)


if __name__ == '__main__':
    rospy.init_node('angleDetection')
    angles = AngleDetection()

    angles.run()
