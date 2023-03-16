#!/usr/bin/env python3
import cv2
import numpy as np
from datetime import datetime, timedelta
import rospy
from std_msgs.msg import Bool
import time

class HSV:
    def __init__(self):
        pass

    def hsvFilter(self, frame):
        """Filters out bgr values
        Args:
            frame: frame provided by camera

        Returns: 
            hue, saturation, value of frame, more accurate than regular rgb
        """

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return hsv

    def masking(self, hsv):
        """Filters out blue values
        Args:
            hsv: hue, saturation, value

        Returns:
            image filtered on blue values
        """

        lower_blue = np.array([101, 50, 38])
        upper_blue = np.array([110, 255, 255])
        maskBlue = cv2.inRange(hsv, lower_blue, upper_blue)
        return maskBlue

    def bitWise(self, mask, frame):
        """converts values to black or white
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            converts each corresponding number in mask and frame to its binary form 
            and then returns either black or white pizxels 
        """
        target = cv2.bitwise_and(frame, frame, mask=mask)
        return target


class Draw:
    def circle(self, mask, frame):
        """Draws circle on objects with a certain color
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            radius and center of circle around the target object
        """
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            radius = 30
            center = (int(cx), int(cy))
            cv2.circle(frame, center, radius, (0, 0, 255), -1)

            return radius, center

        # if no object has been detected -> prevents crash
        center = (200, 200)
        radius = 2
        cv2.circle(frame, center, radius, (0, 0, 255), -1)

        return radius, center

    def square(self, mask, frame):
        """
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            sides of a drawn rectangle around the target object
        """
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            img = cv2.rectangle(frame, (cx, cy), (20, 30), (0, 255, 0), -1)
            a = 20
            b = 30
            if a != 20 | b != 30:
                cv2.rectangle(img, (cx, cy), (20, 30), (0, 0, 0), -1)
            return cx, cy, img

    def contour(self, mask, frame):
        """Contour of image objects
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            radius and center of contour circle around the target object 
        """

        # get contour around object
        contour, hierarchy = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # define biggest contour and its corresponding index
        maxContour = -1
        idxMax = 0

        # find biggest contour and its corresponding index
        for i in range(len(contour)):
            tempContour = cv2.contourArea(contour[i])
            if (maxContour < tempContour):
                maxContour = tempContour
                idxMax = i

        # retrieve center and radius of contour
        if not idxMax:
            # arbitrary value such that the program does not crash if it has not received a value
            contourMax = np.array([[[100, 140]], [[120, 130]]])
        else:
            contourMax = contour[idxMax]

        (x, y), radius = cv2.minEnclosingCircle(contourMax)
        center = (int(x), int(y))  # required to be an int
        radius = int(radius)  # required to be an int

        if radius > 40:
            radius = 40
        # draw contour
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        return radius, center


class Camera:
    def __init__(self):
        """Defines camera related features
        Variables:
            self.vc: opens connection to usb camera. On windows define as 2, on linux define as -1

            self.vs.set: defines width and height of camera capture pop up window

            cv2.namedWindow(): defines name of camera capture pop up window        
        """

        self.vc = cv2.VideoCapture(-1)passwor,

    def checkFrame(self):
        """Function checks if camera opens by trying to get the first frame.
        If false, frame and rval return false. If true rval and frame are set

        Returns:
            rval: Boolean value, which is true if self.vs was able to capture camera image

            frame: image or video captured by camera
        """

        if self.vc.isOpened():  # try to get the first frame
            rval, frame = self.vc.read()
            return rval, frame
        else:
            frame, rval = False
            return rval, frame

    def showImg(self, target):
        """shows alterted hsv imag
        Args:
            target: image that will be shown in the cv2.namedWindow() window

        Returns:
            altered image
        """
        cv2.imshow("preview", target)

    def validation(self):
        """checks if camera was able to open, if not it prints an error message

        Returns:
            rval: Boolean value, which is true if self.vs was able to capture camera image

            frame: image or video captured by camera
        """
        rval, frame = self.checkFrame()
        if (rval or frame) == False:
            print('[ERROR]: Failed to get the first frame')
            return

        return rval, frame


class Overlap:
    def __init__(self):
        self.delay = datetime.today()
        self.overlapping = False

    def circleInCircle(self, radiusContour, radiusCircle, centerContour, centerCircle):
        """Checks if circle is inside of contour
        Args:
            radiusContour: radius of contour around target retrieved from Draw.contour()

            radiusCircle:  radius of circle around target retrieved from Draw.circle()

            centerContour: center of contour in target retrieved from Draw.contour()

            centerCircle: center of circle in target retrieved from Draw.circle()

        Returns:
            move command for the robot with a delay of x seconds such that the command
            cannot be executed again in this time frame
        """
        dist_square = (centerContour[0] - centerCircle[0]) ** 2 \
            + (centerContour[1] - centerCircle[1]) ** 2
        current_time = datetime.today()

        self.overlapping = False

        if (dist_square + radiusCircle ** 2 < radiusContour ** 2 and self.delay <= current_time):
            # overlap
            self.delay = datetime.today() + timedelta(seconds=5)

            # send overlap to true
            self.overlapping = True

            print('overlap')

        print(self.overlapping)
        return self.overlapping


class Run:
    def __init__(self):
        """Runs camera simulation 
        Variables:
            self.hsv: contains all methods and instance variables of the HSV class

            self.draw: contains all methods and instance variables of the Draw class

            self.camera: contains all methods and instance variables of the Camera class

            self.overlap: contains all methods and instance variables of the Overlap class
        """
        rospy.init_node('camera')
        self.pubOverlap = rospy.Publisher('overlap', Bool, queue_size=60)
        self.hsv = HSV()
        self.draw = Draw()
        self.camera = Camera()
        self.overlap = Overlap()

    def runScriptCircle(self):
        """
        Calculates circle and contour (in circle form) of target object and returns an image 
        """
        rval, frame = self.camera.validation()

        while rval:
            start_time = time.time()
            # HSV related
            hsv = self.hsv.hsvFilter(frame)
            mask = self.hsv.masking(hsv)

            # drawing contour of circle and red circle
            radiusContour, centerContour = self.draw.contour(mask, frame)
            radiusCircle, centerCircle = self.draw.circle(mask, frame)

            # check if circle inside contour
            checkOverlap = self.overlap.circleInCircle(
                radiusContour, radiusCircle, centerContour, centerCircle)

            print(checkOverlap)
            self.pubOverlap.publish(checkOverlap)

            self.camera.showImg(frame)
            rval, frame = self.camera.vc.read()

            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                break

            duration = (time.time() - start_time()) * 1000 #in milliseconds

        self.camera.vc.release()
        cv2.destroyWindow("preview")

    def runScriptBitwise(self):
        """
        Calculates bitwise image of the provided image/video
        """
        rval, frame = self.camera.validation()

        while rval:
            hsv = self.hsv.hsvFilter(frame)
            mask = self.hsv.masking(hsv)
            target = self.hsv.bitWise(mask, frame)

            # show the hsv image
            self.camera.showImg(target)

            rval, frame = self.camera.vc.read()
            key = cv2.waitKey(20)
            if key == 27:  # exit on ESC
                break

        self.vc.release()
        cv2.destroyWindow("preview")


run = Run()
run.runScriptCircle()
# run.runScriptBitwise()
