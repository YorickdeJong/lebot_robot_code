#!/usr/bin/env python3

import motor
import numpy as np
import rospy

class Actions:
    def __init__(self):
        self.motor = motor.Motor()
        self.checkGamma = True
        self.checkDistToWall = True
        self.checkTurning = True

    def driveForward(self, vel):
        """Sets x velocity in robot frame to be positive
        Args:
            velocity in the x direction of the vehicle in the robot frame
        """
        self.motor.set_x_linear_vel(vel)
    
    def driveBackward(self, vel):
        """Sets x velocity in robot frame to be negative
        Args:
            velocity in the x direction of the vehicle in the robot frame
        """
        self.motor.set_x_linear_vel(-vel)

    def correctionAngle(self, gamma, sign):
        """Vehicle turns untill gamma is below a certain threshold
        Args:
            sign: Direction in which has to be turned

        Returns:
            Correct course measured against nearest object 
        """

        self.motor.set_z_angular_vel(sign * 0.6)
        threshold = 5/90  # get robot back on the correct path

        if gamma > threshold:  # keeps turning untill gamma is smaller than threshold
            self.checkGamma = True
            print('correction gamma is: ' + str(gamma))
            return

        #ends loop
        self.checkGamma = False
        self.motor.set_z_angular_vel(0)
        return 

    def correctionDistance(self, distToWall):
        """Vehicle drives backwards untill the distance is below a threshold

        Returns: vehicle that is a sufficient distance away from object
        """

        threshold = 0.5
        self.driveBackward(-0.8)

        if distToWall < threshold:
            self.checkDistToWall = True
            print('correction distance is: ' + str(distToWall))
            return

        #ends loop
        self.checkDistToWall = False
        self.driveBackward(0)
        return 

    def turning90Degrees(self, gamma, checkOverlap):
        """When a target object is scanned by the camera, the vehicle turns by
        a certain threshold

        Returns: turned vehicle compared to target object
        """
        if checkOverlap:  # Check if overlap is true
            print('overlap')
            threshold = 0.47  # turns 0 degrees put on 0.47 because of accuracy
            self.motor.set_z_angular_vel(0.6)

            if gamma < threshold:  # gamma must be bigger than threshold -> turn complete
                self.checkGamma = True
                print('gamme turning: ' + str(gamma))
                return
        
        #ends loop
        self.motor.set_z_angular_vel(0)
        self.checkGamma = False
        return