#!/usr/bin/env python3

import navigationData
import numpy as np
import rospy
from driver_bot_pkg.msg import ObjectAngle
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import motor

class RunScript:
    def __init__(self):
        self.subObjectAngle = rospy.Subscriber(
            'objectPosition', ObjectAngle, self.objectAngleCallback)
        self.neirestDistance = rospy.Subscriber(
            'minDistance', Float64, self.minDistanceCallback)
        self.subGamma = rospy.Subscriber('gamma', Float64, self.gammaCallback)
        self.subdist = rospy.Subscriber(
            'distance', Float64, self.distToWallCallback)
        self.subOverlap = rospy.Subscriber(
            'overlap', Bool, self.overlapCallback)

        self.motor = motor.Motor()
        self.navigation = navigationData.Actions()
        self.gamma = 1  # arbitrary value
        self.distToWall = 20  # arbitrary value
        self.checkOverlap = False
        self.rightSide = 0
        self.angle_new = 0
        self.sign = 0
        self.minDistance = 10

    def objectAngleCallback(self, msg):
        """Callback for objectAngle message defined in angle_node/AngleDetection()"""
        self.rightSide = msg.rightSide
        self.angle_new = msg.angle_new
        self.sign = msg.sign

    def gammaCallback(self, msg):
        """reads gamma callback messages from 'gamma'
        Args:
            msg: retrieves data from 'gamma' topic, which contains the angle between the robot frame and 
            the world frame

        returns:
            gamma, which is the angle between robot frame and world frame
        """
        self.gamma = msg.data

    def distToWallCallback(self, msg):
        """reads ranges call back message from 'distance'
        Args:
            msg: retrieves data from 'distance' topic, which contains the distance infront 
            of the vehicle, measured by the RPLIDAR

        returns:
            distance between 11/6 and 1/6 pi and 0.15m < d < 12m
        """
        self.distToWall = msg.data

    def overlapCallback(self, msg):
        """reads a call back message from 'scan'
        Args:
            msg: retrieves data from 'overlap' topic, which contains a boolean value that is true if a defined circle is inside
            of a circle contour and for a target object. The object is measured by a camera, which data operations are defined in
            camera.py

        returns:
            boolean value for overlap of circle and contour circle for a target object. if Camera is not connect, default is False
        """
        try:
            self.checkOverlap = msg.data
        except:
            print('camera is not on or data is not being received well')
            self.checkOverlap = False

    def minDistanceCallback(self, msg):
        """Gets data from minDistance topic defined in distance_node/rplidar()
        Args:
            msg: contains distance to neirest object
        """
        self.minDistance = msg.data

    def validation(self):
        if self.rightSide != False and self.rightSide != True :
            print('[ERROR] rigthSide in RunScript has not been set correctly')
            print('[VALUE]: ' + str(self.rightSide))

        if self.angle_new == 0:
            print('[ERROR] angle_new in RunScript has not been set correctly')
            print('[VALUE]: ' + str(self.angle_new))

        if self.sign == 0:
            print('[ERROR] sign in RunScript has not been set correctly')
            print('[VALUE]: ' + str(self.sign))

        if self.minDistance == 10:
            print('[ERROR] minDistance in RunScript has not been set correctly')
            print('[VALUE]: ' + str(self.minDistance))

        if self.distToWall == 20:
            print('[ERROR] WALL DISTANCE DATA HAS NOT BEEN RECEIVED')
            print('[VALUE]: ' + str(self.distToWall))

        if self.gamma == 1:
            print('[ERROR] GAMMA HAS NOT BEEN RECEIVED')
            print('[VALUE]: ' + str(self.gamma))

    def AdjustPos(self):
        """Calculates orientation of the robot compared to the world frame and adjusts
        its behaviour accordingly

        Returns:
            command for the robot to follow if walls/ objects are near. A while loop is also included
            to prevent to robot from getting stuck inbetween objects. It scans if there are objects
            infront of it in a range of 0.4m, if not, it moves forward
        """
        error = 20 / 90  # 25 degree angle
        minDist = 0.50
        minDistToWall = 0.4  # minDist > minDistToWall must hold

        self.validation()
        while self.distToWall < 0.4:

            # checks position of the wall, direction of the bot, error and distance to the wall to determine the appropriate operation
            if (not self.rightSide and self.sign == -1 and (self.gamma > error or self.minDistance < minDist)):  # 1
                # driving towards left wall -> prevent by turning right ##Not good
                print('Moving towards left wall, adjusting right as to not hit the wall')
                while self.navigation.checkGamma:
                    self.navigation.correctionAngle(self.gamma, 1)
                self.navigation.checkGamma = True

            if (not self.rightSide and self.sign == 1 and self.gamma > error and self.minDistance < minDist):  # 2
                # turn away from left wall -> turn left if turn is to big
                print('Moving away from left wall, adjust left as to stay on course!')
                while self.navigation.checkGamma:
                    self.navigation.correctionAngle(self.gamma, -1)
                self.navigation.checkGamma = True

            if (self.rightSide and self.sign == -1 and self.gamma > error and self.minDistance < minDist):  # 3
                # turn away from right wall -> turn right if turn is to big ##Not good
                print('Moving away from right wall, adjust right as to stay on course!')
                while self.navigation.checkGamma:
                    self.navigation.correctionAngle(self.gamma, 1)
                self.navigation.checkGamma = True

            if (self.rightSide and self.sign == 1 and self.gamma > error and self.minDistance < minDist):  # 4
                # driving towards the right wall -> prevent by turning left
                print(
                    'Moving towards right wall, adjusting left as to not hit the wall!')
                while self.navigation.checkGamma:
                    self.navigation.correctionAngle(self.gamma, -1)
                self.navigation.checkGamma = True

            if self.distToWall < minDistToWall:
                # if robot is driving straight a head to object
                print(
                    'moving straight ahead to object, prevent by going back and turning')
                if self.rightSide:
                    sign = -1
                else:
                    sign = 1

                while self.navigation.checkDistToWall:
                    self.navigation.correctionDistance(self.distToWall)
                self.navigation.checkDistToWall = True

                while self.navigation.checkGamma:    
                    self.navigation.correctionAngle(self.gamma, sign)
                self.navigation.checkGamma = True

        # check for target object
        while self.navigation.checkGamma:
            self.navigation.turning(self.gamma, self.checkOverlap)

        # if distance to object infront of the robot is larger than 40 cm, move forward
        self.motor.set_x_linear_vel(1)
        print('moving forward')

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.AdjustPos()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('runScript')
    runScript = RunScript()
    runScript.run()
