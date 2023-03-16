#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# own packages
import driver_bot_pkg.src.lidar_node as lidar_node
import FrameAnalyses
import PerceptionInterface


class ObjectDetection(PerceptionInterface):
    def __init__(self):
        """Runs camera simulation 
        Variables:
            self.filter: containes filter methods hsv, maska and bitwise

            self.draw: contains draw functions such as contour, circle and square

            self.camera: contains data about frame and checks the validaty

            self.overlap: contains information about overlap between contour and image
        """
        rospy.init_node('camera')
        self.pubOverlap = rospy.Publisher('overlap', Bool, queue_size=60)

        self.camera = lidar_node.Camera()
        self.filter = FrameAnalyses.Filter()
        self.draw = FrameAnalyses.Draw()
        self.overlap = FrameAnalyses.Overlap()

    def dataAnalyses(self):
        """Calculates hue saturation value (hsv), masking for a blue color
        and retrieves the radius and center of a contour and a circle 

        """
        # analysing image
        hsv = self.filter.hsvFilter(self.frame)
        maskingBlue = self.filter.maskingBlueFilter(hsv)

        # drawing contour around object and red circle inside object
        radiusContour, centerContour = self.draw.contour(
            maskingBlue, self.frame)
        radiusCircle, centerCircle = self.draw.circle(maskingBlue, self.frame)
        return radiusContour, centerContour, radiusCircle, centerCircle

    def detection(self):
        """
        Calculates circle and contour (in circle form) of target object and returns an image 
        """
        rval, frame = self.camera.validation()

        while rval:
            # Cicle detection
            checkBlue = self.detectionCircle()  # can add more shapes here

            # publishes data on the overlap topic
            self.pubOverlap.publish(checkBlue)

            self.camera.showImg(frame)
            rval, frame = self.camera.vc.read()

            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                break

        self.camera.vc.release()
        cv2.destroyWindow("preview")

    # ---------support functions----------#
    def detectionCircle(self):

        radiusContour, centerContour, radiusCircle, centerCircle = self.dataAnalyses()

        # check if circle inside contour
        checkOverlap = self.overlap.circleInCircle(
            radiusContour, radiusCircle, centerContour, centerCircle)

        return checkOverlap


camera = ObjectDetection()
camera.detection()
