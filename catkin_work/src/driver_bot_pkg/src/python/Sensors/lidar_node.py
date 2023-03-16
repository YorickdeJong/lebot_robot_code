#!/usr/bin/env python3

from abc import ABC, abstractmethod
import cv2
import rospy
from sensor_msgs.msg import LaserScan
from driver_bot_pkg.msg import LidarData


class SensorInterface(ABC):
    """Interface shows common functions of Camera and rpLidar class"""
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def read(self, data):
        pass

    @abstractmethod
    def validate(self):
        pass



class RPLidar():
    # ---------overriding----------#
    def __init__(self):

        self.dist = [20]  # value greater than 12 to check if data has been received well
        # value greater than regular received value to check validation
        self.angle_increment = 1
        self.subLidar = rospy.Subscriber(
            'scan', LaserScan, self.read)
        self.pubValData = rospy.Publisher('rplidar', LidarData, queue_size=10)

    def read(self, msg):
        """Returns multiple callbacks"""
        self.angleCallback(msg)
        self.distanceCallback(msg)
        self.validate()

    def validate(self):
        if self.dist[0] == 20:
            print('[ERROR] DISTANCE DATA HAS NOT BEEN RECEIVED')

        if self.angle_increment == 1:
            print('[ERROR] ANGLE_INCREMENT HAS NOT BEEN RECEIVED')

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publishValidatedData()
            rate.sleep()

    # ---------support functions----------#
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
        print('sensor node angle_increment: ' + str(self.angle_increment))

    def publishValidatedData(self):
        """Publishes altered message with distance and angle_increment"""
        msg = LidarData()
        msg.distance = self.dist
        msg.angle_increment = self.angle_increment
        self.pubValData.publish(msg)


if __name__ == '__main__':
    rospy.init_node('Distance')
    data = RPLidar()
    data.run()

