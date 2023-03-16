#!/usr/bin/env python3

import matplotlib 
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np
from driver_bot_cpp.msg import distanceVelocity
import rospy


class PlotCalibrationData:
    """Class plots data received from the calibration class that is send on the 'distanceVelocity' Topic"""
    def __init__(self):
        self.m_subDistanceVelocity = rospy.Subscriber("distanceVelocity", distanceVelocity, self.distanceVelocityCallback)
        self.distance = 10
        self.time = 1
        self.velocity = 20
        self.motorSpeed = -1
        self.plotCompleted = False

    def distanceVelocityCallback(self, msg):
        """saves data from 'distanceVelocity' topic
        Args:
            msg: contains time, distance and velocity arrays of every calibration step
        """
        self.time = msg.time 
        self.timeVelocity = msg.timeVelocity
        
        self.distance = msg.distance
        self.velocity = msg.velocity
        self.motorSpeed = msg.motorPower
        checkValidation = self.validateCallback()

        if not checkValidation:
            print("VARIABLES HAVE NOT BEEN SET CORRECTLY")

    def validateCallback(self):
        """Validates if data is received or not"""
        

        if self.distance == 10:
            checkValidationDistance = False        
        else:
            checkValidationDistance = True

        if self.time == 1:
            checkValidationTime = False
        else:
            checkValidationTime = True

        if self.velocity == 20:
            checkValidationVelocity = False
        else:
            checkValidationVelocity = True

        if self.motorSpeed == -1:
            checkValidationVelocity = False
        else:
            checkValidationVelocity = True
        
        if checkValidationDistance and checkValidationTime and checkValidationVelocity:
            return True

        return False         

    def resetData(self):
        """Resets all values to their initial value"""
        self.distance = 10
        self.time = 1
        self.velocity = 20
        self.motorSpeed = -1
        self.plotCompleted = False

    def singlePlot(self, index, x, y, Label, yLabel, Title, saveFig):
        """Creates plot of 'distanceVelocity' input"""
        #distance
        plt.figure(index)
        plt.plot(x, y, label = str(Label))
        plt.xlabel("time")
        plt.ylabel(yLabel)
        plt.title(Title)
        plt.savefig(saveFig + str(index) + ".png")
        plt.show()

    def plotData(self, index):
        """Function plots data for distance vs time, and velocity vs time
        returns True if plot has been created, returns False if not"""

        if self.validateCallback():

            self.singlePlot(index, self.time[1::], self.distance[1::], self.motorSpeed, "distance", 
            "Plot distance vs time", "src/driver_bot_cpp/nodes/src/Calibrate/figures/distance/distance_vs_time")
            self.singlePlot(10 + index, self.timeVelocity[1::], self.velocity[1::], self.motorSpeed, "velocity", 
            "Plot velocity vs time", "src/driver_bot_cpp/nodes/src/Calibrate/figures/velocity/velocity_vs_time")

            return True

        print("################ PYTHON PLOT ###############")
        print("DATA HAS NOT BEEN PLOTTED")
        print("############################################")
        return False
        
    def run(self):
        rate = rospy.Rate(10)
        
        for i in range(10):
            while not rospy.is_shutdown() and not self.plotCompleted:
                self.plotCompleted = self.plotData(i) 
                rate.sleep()
            self.resetData()



if __name__ == '__main__':
    rospy.init_node('plotData')
    plotDistanceVelocity = PlotCalibrationData()
    plotDistanceVelocity.run()