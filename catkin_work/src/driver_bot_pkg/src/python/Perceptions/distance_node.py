#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from driver_bot_pkg.msg import LidarData

# own packages
import Sensors.lidar_node as lidar_node
import Perceptions.PerceptionInterface as PerceptionInterface


class DistanceDetection():
    def __init__(self):

        self.pubDistance = rospy.Publisher('distance', Float64, queue_size=10)
        self.pubMinDistance = rospy.Publisher(
            'minDistance', Float64, queue_size=10)
        self.subLidar = rospy.Subscriber(
            'rplidar', LidarData, self.lidarCallback)

        self.angle_increment = 1
        self.dist = 10

    def lidarCallback(self, msg):
        """Callback function containing data from rplidar
        Args:
            msg: contains data for distance and angle_increment
        """
        self.angle_increment = msg.angle_increment
        self.dist = msg.distance

    def dataAnalyses(self):
        """Produces an array of of values with increment self.angle_increment for one round
        trip between 0 and 2 pi.We reverse the list since the rpLidar scanner starts at 2pi
        and ends at 0 pi"""

        numberArray = np.linspace(
            0, 2, 2 * np.pi / self.angle_increment)  # idx of angles
        return np.flip(numberArray, 0)

    def detection(self):
        """Breaks up distance object from 0 to 2pi into a specific angle range
        Variables:
            angle1: lower bound of angle range
            angle2: upper bound of angle range

        Returns:
            array of distance values for angle1 < angle < angle2
        """
        angle1 = 1/6
        angle2 = 11/6

        angleArray = self.dataAnalyses()
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

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            distanceRange = self.detection()
            minDistance = self.minimumDistance(self.dist)

            self.publishDistanceRange(distanceRange)
            self.publishMinDistance(minDistance)

            rate.sleep()

    # ---------Additional functionalities----------#
    def publishDistanceRange(self, distance):
        """Publishes distance for a specific angle range"""
        self.pubDistance.publish(distance)
        print('dist range: ' + str(distance))

    def publishMinDistance(self, distance):
        """Publishes minimum distance for 0 to 2 pi"""
        self.pubMinDistance.publish(distance)
        print('min dist is: ' + str(distance))

    # ---------support functions----------#

    def minimumDistance(self, distance):
        """Function calculates minimum distance to object
        Args:
            distance: array containing distance to all objects between angle1 and angle2 for 0.15m < d < 12m

        Returns:
            minimum distance of the provided distance array
        """
        distMin = np.min(distance)
        return distMin

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


if __name__ == '__main__':
    rospy.init_node('distanceDetection')
    dist = DistanceDetection()
    dist.run()
