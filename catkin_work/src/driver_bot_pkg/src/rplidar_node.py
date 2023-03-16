#!/usr/bin/env python3

import rospy
import time
import numpy as np
import motor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CallBack:
    """Measures distance towards the nearest object and returns that distance with the corresponding angle. Hereafter the velocity
    and direction of the robot is changed."""

    def __init__(self):
        rospy.init_node('lidar')

        # callback results
        self.checkOverlap = False
        self.vel = 0
        self.dist = 1  # dist to all objects
        self.angle_increment = 1  # MUST BE ANY VALUE ABOVE ZERO -> ERROR OTHERWISE
        self.idx_range = 1  # MUST BE ANY VALUE ABOVE ZERO -> ERROR OTHERWISE

        # Subscribers and publishers
        self.subLidar = rospy.Subscriber(
            'scan', LaserScan, self.scanCallback)
        self.subVel = rospy.Subscriber('vel', Twist, self.velCallback)
        self.subOverlap = rospy.Subscriber(
            'overlap', Bool, self.overlapCallback)

        # objects
        self.motor = motors.Motor()

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

    def velCallback(self, msg):
        """reads a call back message from 'scan'
        Args:
            msg: retrieves data from 'scan' topic, which contains the distance at a certain angle, measured by the RPLIDAR

        returns:
            minimum distance and the corresponding angle
        """
        self.vel = msg.data
        # print(self.vel(str))

    def distanceCallback(self, msg):
        """reads ranges call back message from 'scan'
        Args:
            msg: retrieves data from 'scan' topic, which contains the distance at a certain angle, measured by the RPLIDAR

        returns:
            distance between 0 and 2 pi and 0.15m < d < 12m
        """
        self.dist = msg.ranges

    def angleCallback(self, msg):
        """reads angle_increment callback messages from 'scan'
        Args:
            msg: retrieves data from 'scan' topic, which contains the distance at a certain angle, measured by the RPLIDAR

        returns:
            angle increment in between measured scan
        """
        self.angle_increment = msg.angle_increment


    def scanCallback(self, msg):
        """Returns multiple callbacks"""
        self.angleCallback(msg)
        self.distanceCallback(msg)


class Distance(CallBack):
    def __init__(self):
        super.__init__()
        
    def minimumDistance(self, distance):
        """Function calculates minimum distance to object
        Args:
            distance: array containing distance to all objects between angle1 and angle2 for 0.15m < d < 12m

        Returns:
            minimum distance of the provided distance array
        """
        distMin = np.min(distance)
        return distMin

    def RangeAngleArg(self):
        """Produces an array of of values with increment self.angle_increment for one round
        trip between 0 and 2 pi.We reverse the list since the rpLidar scanner starts at 2pi
        and ends at 0 pi"""

        numberArray = np.linspace(
            0, 2, 2 * np.pi / self.angle_increment)  # idx of angles
        return np.flip(numberArray, 0)

    def distAtAngleRange(self, angle1, angle2):
        """Breaks up distance object from 0 to 2pi into a specific angle range
        Args:
            angle1: lower bound of angle range
            angle2: upper bound of angle range

        Returns:
            array of distance values for angle1 < angle < angle2
        """

        angleArray = self.RangeAngleArg()
        distanceIdx = np.where((angleArray > angle1) & (angleArray < angle2))[
            0]  # get unwanted elements

        indicesArray = np.arange(len(angleArray))
        # delete unwanted elements

        angleIdx = np.delete(indicesArray, distanceIdx)
        try:
            distanceRange = self.getIndices(angleIdx, self.dist)
        except:
            distanceRange = [1]

        return self.minimumDistance(distanceRange)

    def getIndices(self, arrayIdx, arrayObj):
        """Retrieves values from arrayObj based on indices provided in arrayIdx
        Args:
            arrayidx: array containing wanted indices

            arrayObj: array containing objects
        Returns: Array with wanted values, based on elements provided in arrayIdx
        """
        obj = np.zeros(len(arrayIdx) - 1)
        arrayIdx = arrayIdx[0: len(arrayIdx) - 1]

        for idx, value in enumerate(arrayIdx):
            obj[idx] = arrayObj[value]

        return obj


class Angle(CallBack):
    def __init__(self):
        super.__init__()

    def minimumDistanceArg(self):
        """Calculates index for a minimum distance"""
        return np.argmin(self.dist)

    def angleCalc(self):

        unit_circle = 2*np.pi
        increments_in_circle = unit_circle/self.angle_increment
        angle = self.minimumAngleArg()/increments_in_circle * 2
        # print('angle is: ' + str(angle) + " pi")
        return angle

    def angleOrientation(self, angle):
        """Check where the wall of the robot is and adjust the angle and
        make sure that new angle is always smaller than 1/2 for gamma calc
        Args:
            angle: angle at which minimum distance occurs

        Returns:
            rightSide: True if wall is on the right side of the vehicle
            angle_new: adjusted angle for position of the wall compared to the vehicle
            sign: contains direction of the vehicle's y velocity in world coordinates
        """
        if 1/5 < angle < 1/2:
            # moving towards left wall
            rightSide = False
            angle_new = angle
            sign = -1

        if 1/2 < angle < 1:
            # moving away from left wall
            rightSide = False
            angle_new = angle - 1/2
            sign = 1

        if 1 < angle < 3/2:
            # moving away from right wall
            angle_new = angle - 1
            rightSide = True
            sign = -1

        if 3/2 < angle < 9/5:
            # moving towards right wall
            angle_new = angle - 3/2
            rightSide = True
            sign = 1

        if angle <= 1/5 or angle >= 9/5:
            # Will not get triggered by wall
            # Prevents getting stuck
            rightSide = False
            angle_new = 0
            sign = 0

        return rightSide, angle_new, sign

    def worldVelocities(self, angle, xVelRobot, sign):
        """Calcultas velocity of robot in world frame
        Args:
            angle: angle of robot compared to nearest object

            xVelRobot: x velocity of robot

            sign: direction of the y velocity of the robot in world frame

        returns:
            velocities of robot in world frame
        """
        # calculate velocities in world frame
        xVelWorld = abs(np.cos(angle * np.pi)) * xVelRobot
        # since rplidar has right handed coordinate system (pos x begins at 2 pi) we need a minus infront
        yVelWorld = sign * abs(np.sin(angle * np.pi)) * xVelRobot

        return xVelWorld, yVelWorld

    def gammaAngle(self, angle_new):
        """angle between robot frame and world frame"""
        return 1/2 - angle_new

#############navigation Interface
    def correctionAmount(self, gamma, sign):
        self.motor.set_z_angular_vel(sign * 0.6)
        threshold = 5/90  # get robot back on the correct path

        while gamma > threshold:
            print('gamma correction: ' + str(gamma))
            angle = self.angleCalc()
            rightSide, angle_new, sign = self.angleOrientation(angle)
            gamma = self.gammaAngle(angle_new)

        self.motor.set_z_angular_vel(0)

    def correctionDistance(self, distToWall):
        threshold = 0.5
        self.motor.set_x_linear_vel(-1)

        while distToWall < threshold:
            print(distToWall)
            distToWall = self.distAtAngleRange(1/5, 9/5)

    def turning(self):
        threshold = 0.47  # turns 0 degrees put on 0.47 because of accuracy
        angle = self.angleCalc()
        rightSide, angle_new, sign = self.angleOrientation(angle)
        # define angle between robot frame and world frame
        gamma = self.gammaAngle(angle_new)
        self.motor.set_z_angular_vel(0.6)
        print('turning right')

        while gamma < threshold:
            print('gamme overlap: ' + str(gamma))
            angle = self.angleCalc()
            rightSide, angle_new, sign = self.angleOrientation(angle)
            gamma = self.gammaAngle(angle_new)

    def detectOverlap(self):
        if self.checkOverlap:
            print('overlap')
            self.turning()

    def adjustPos(self, xVelRobot):
        """Calculates orientation of the robot compared to the world frame and adjusts
        its behaviour accordingly
        Args:
            xVelRobot: x velocity of robot in robot frame

        Returns:
            command for the robot to follow if walls/ objects are near. A while loop is also included
            to prevent to robot from getting stuck inbetween objects. It scans if there are objects
            infront of it in a range of 0.4m, if not, it moves forward
        """
        error = 20 / 90  # 25 degree angle

        minDist = 0.50
        minDistToWall = 0.4  # minDist > minDistToWall must hold!
        # distance to the wall, measured in an angle range
        distToWall = self.distAtAngleRange(1/6, 11/6)
        print('range ahead first: ' + str(distToWall))
        while distToWall < 0.4:
            angle = self.angleCalc()

            # angle between 1/5 and 9/5 rightsided
            distToNearestObject = self.minimumDistance(self.dist)
            rightSide, angle_new, sign = self.angleOrientation(angle)
            xVelWorld, yVelWorld = self.worldVelocities(angle, xVelRobot, sign)

            # define angle between robot frame and world frame
            gamma = self.gammaAngle(angle_new)
            print('gamma is:' + str(gamma))

            # checks position of the wall, direction of the bot, error and distance to the wall to determine the appropriate operation
            if (not rightSide and sign == -1 and (gamma > error or distToNearestObject < minDist)):  # 1
                # driving towards left wall -> prevent by turning right ##Not good
                print('Moving towards left wall, adjusting right as to not hit the wall')
                self.correctionAmount(gamma, 1)

            if (not rightSide and sign == 1 and gamma > error and distToNearestObject < minDist):  # 2
                # turn away from left wall -> turn left if turn is to big
                print('Moving away from left wall, adjust left as to stay on course!')
                self.correctionAmount(gamma, -1)

            if (rightSide and sign == -1 and gamma > error and distToNearestObject < minDist):  # 3
                # turn away from right wall -> turn right if turn is to big ##Not good
                print('Moving away from right wall, adjust right as to stay on course!')
                self.correctionAmount(gamma, 1)

            if (rightSide and sign == 1 and gamma > error and distToNearestObject < minDist):  # 4
                # driving towards the right wall -> prevent by turning left
                print(
                    'Moving towards right wall, adjusting left as to not hit the wall!')
                self.correctionAmount(gamma, -1)

            if distToWall < minDistToWall:
                # if robot is driving straight a head to object
                print(
                    'moving straight ahead to object, prevent by going back and turning')
                if rightSide:
                    sign = -1
                else:
                    sign = 1
                self.correctionDistance(distToWall)
                self.correctionAmount(gamma, sign)

            # Recalculate distInAngle
            distToWall = self.distAtAngleRange(1/6, 11/6)
            print('range ahead: ' + str(distToWall))

        # if distance to object infront of the robot is larger than 40 cm, move forward
        self.motor.set_x_linear_vel(1)
        print('moving forward')

    def run(self):
        """
        Defines an initial direction and speed. If the distance to an object is less than 0.3, the car moves in another
        direction for x amount of time. If ctrl + c is pressed, the while loop exits.
        """
        rate = rospy.Rate(60)
        self.motor.set_x_linear_vel(0.4)

        while not rospy.is_shutdown():

            self.adjustPos(self.motor.xLinearVel)

            rate.sleep()
        self.motor.initial_val()


dist = Distance()
dist.run()
