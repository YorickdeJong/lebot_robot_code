#!/usr/bin/env python3

import matplotlib 
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np
from driver_bot_cpp.msg import distanceVelocity
import rospy
import subprocess
import sys
import os
import signal
import cv2
from cv_bridge import CvBridge
from driver_bot_cpp.msg import images


sys.path.append('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/')
from support_classes.plot_data import LidarPlot, SonarPlot, EncoderPlot


def get_num_publishers(topic_name):
    cmd = ['rostopic', 'info', topic_name]
    output = subprocess.check_output(cmd).decode('utf-8')
    num_publishers = output.count('Publisher: ')
    return num_publishers


class PlotData:
    """Class plots data received from various nodes which include lidar node, sonar node, calibration node, etc"""
    
    def __init__(self):
        self.subLidar = rospy.Subscriber("lidarData", distanceVelocity, self.movementCallback)
        #TODO add other subscribers too.

        self.time = []
        self.timeVelocityPlot = []
        self.distance = []
        self.velocity = []
        self.type = ""
        self.lastSubscriber = ""
        
        self.lidar_plot = LidarPlot()
        self.sonar_plotter = SonarPlot()
        self.encoder_plotter = EncoderPlot()

        # Create a publisher for the plot image
        self.pubImages = rospy.Publisher('movement_plot', images, queue_size=10)

        # Create a CvBridge object for converting between OpenCV images and ROS image messages
        self.bridge = CvBridge()

    def movementCallback(self, msg):
        """saves data from 'distanceVelocity' topic
        Args:
            msg: contains time, distance, velocity and measurement device arrays of every calibration step
        """
        self.time = msg.time 
        self.timeVelocityPlot = msg.timeVelocityPlot 
        self.distance = msg.distance
        self.velocity = msg.velocity
        self.type = msg.type

        if len(msg.distance) != len(msg.time):
            print('Plotting error: distance and velocity arrays are not of the same length')
            return

        if self.type == 'lidar':
            self.lastSubscriber = 'lidar'
        elif self.type == 'sonar':
            self.lastSubscriber = 'sonar'
        elif self.type == 'encoder':
            self.lastSubscriber = 'encoder'

    def resetData(self):
        """Resets data to default values"""
        self.time = []
        self.distance = []
        self.velocity = []
        self.type = ""

    def dataReceived(self):
        return len(self.distance) > 0

    def sendPlotImage(self):
        """Generates and sends the plot image as a ROS message"""
        if self.lastSubscriber == 'lidar':
            if self.dataReceived():
                self.lidar_plot.plotDistanceTime(self.time, self.distance)
                self.lidar_plot.plotVelocityTime(self.timeVelocityPlot,  self.velocity)
                self.resetData()
                
                imageVel = plt.imread('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/nodes/src/plots/images/distance_data_lidar.png')
                imageAcc = plt.imread('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/nodes/src/plots/images/velocity_data_lidar.png')
                # Convert the image to a ROS message and publish it
                ros_image_vel = self.bridge.cv2_to_imgmsg(imageVel, encoding='passthrough')
                ros_image_acc = self.bridge.cv2_to_imgmsg(imageAcc, encoding='passthrough')
                image_array = images()
                image_array.images = [ros_image_vel, ros_image_acc]
                image_array.data_type = "lidar"
                
                
                self.pubImages.publish(image_array)
                print('published Lidar plot')
                # Exit the script
                #rospy.signal_shutdown('plot_node shutting down')

    def run(self):
        """Runs the plot node"""
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown(): 
            self.sendPlotImage()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('plot_node')
    plot = PlotData()
    plot.run()
    #os.kill(os.getpid(), signal.SIGINT)