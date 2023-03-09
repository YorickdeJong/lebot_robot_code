#!/usr/bin/env python3

from abc import ABC, abstractmethod
import cv2
import rospy
from sensor_msgs.msg import LaserScan
from driver_bot_pkg.msg import LidarData

class Camera():
    # ---------overriding----------#
    def __init__(self):
        """Defines camera related features
        Variables:
            self.vc: opens connection to usb camera. On windows define as 2, on linux define as -1

            self.vs.set: defines width and height of camera capture pop up window

            cv2.namedWindow(): defines name of camera capture pop up window        
        """
        self.vc = cv2.VideoCapture(-1)
        self.vc.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
        self.vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)
        cv2.namedWindow("preview")
        #self.pubValData = Publisher('cameraValidation', )

    def read(self, data):
        """shows alterted hsv imag
        Args:
            target: image that will be shown in the cv2.namedWindow() window

        Returns:
            altered image
        """
        cv2.imshow("preview", data)

    def validation(self):
        """Function checks if camera opens by trying to get the first frame.
        If false, frame and rval return false and an error message is printed. 
        If true rval and frame are set

        Returns:
            rval: Boolean value, which is true if self.vs was able to capture camera image

            frame: image or video captured by camera
        """

        if self.vc.isOpened():  # try to get the first frame
            rval, frame = self.vc.read()

        else:
            frame, rval = False
            print('[ERROR]: Failed to get the first frame')

        return rval, frame