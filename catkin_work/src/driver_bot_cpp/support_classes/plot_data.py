import matplotlib 
matplotlib.use('agg')
import matplotlib.pyplot as plt
import numpy as np
from driver_bot_cpp.msg import distanceVelocity
import rospy


class LidarPlot:
    def __init__(self):
        pass
    
    def plotDistanceTime(self, x, y):
        """Plots distance versus time of lidar data"""
        plt.figure(1)
        plt.plot(x, y, 'ro-', label = 'velocity', markersize=4, linewidth=1)
        plt.title("Lidar data plot")
        plt.xlabel("time")
        plt.ylabel("distance")
        plt.legend()
        plt.show()
        plt.savefig('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/nodes/src/plots/images/distance_data_lidar.png', 
                    bbox_inches='tight', pad_inches=0.1, dpi=300)

    def plotVelocityTime(self, x, y):
        """Plots velocity versus time of lidar data"""
        plt.figure(2)
        plt.plot(x, y, 'bo-', label = 'acceleration', markersize=4, linewidth=1)
        plt.title("Lidar data plot")
        plt.xlabel("time")
        plt.ylabel("velocity")
        plt.legend()
        plt.show()
        plt.savefig('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/nodes/src/plots/images/velocity_data_lidar.png',
                    bbox_inches='tight', pad_inches=0.1, dpi=300)

class SonarPlot:
    def __init__(self):
        pass
    
    #TODO: ADD MOTOR SPEED TO THE MESSAGE TOPIC
    def plotDistanceTime(self, x, y): #Mistake here
        """Plots distance versus time of sonar data"""
        plt.plot(x, y, 'ro', label = 'velocity')
        plt.title("Sonar data plot")
        plt.xlabel("time")
        plt.ylabel("distance")
        plt.legend()
        plt.show()
        plt.savefig('/home/ubuntu/Documents/LeBot/catkin_work/src/driver_bot_cpp/nodes/src/plots/images/distance_data_sonar.png')

    def plotVelocityTime(self, x, y):
        """Plots velocity versus time of sonar data"""
        plt.plot(x, y, 'ro', label = 'acceleration')
        plt.title("Sonar data plot")
        plt.xlabel("time")
        plt.ylabel("velocity")
        plt.legend()
        plt.show()
        plt.savefig('src/driver_bot_cpp/nodes/src/plots/images/velocity_data_sonar.png')


class EncoderPlot:
    def __init__(self):
        pass
    
    def plotDistanceTime(self, x, y):
        """Plots distance versus time of encoder data"""
        plt.plot(x, y, 'ro', label = 'velocity')
        plt.title("Encoder data plot")
        plt.xlabel("time")
        plt.ylabel("distance")
        plt.legend()
        plt.show()
        plt.savefig('src/driver_bot_cpp/nodes/src/plots/images/distance_data_encoder.png')

    def plotVelocityTime(self, x, y):
        """Plots velocity versus time of encoder data"""
        plt.plot(x, y, 'ro', label = 'acceleration')
        plt.title("Encoder data plot")
        plt.xlabel("time")
        plt.ylabel("velocity")
        plt.legend()
        plt.show()
        plt.savefig('src/driver_bot_cpp/nodes/src/plots/images/velocity_data_encoder.png')