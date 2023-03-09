#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <math.h>  
#include <tuple> 
#include <iostream>
#include <string>

#include "std_msgs/Bool.h"
#include "driver_bot_cpp/lidar.h"
#include "driver_bot_cpp/distanceVelocity.h"
#include "../../support_classes/include/distance_operations.h"
#include "../../magicNumbers/magicNumbers.h"

class DataTransformationPlot{

    public:
        DataTransformationPlot();
        ~DataTransformationPlot() = default;

        void DistanceCallback(const driver_bot_cpp::lidar::ConstPtr& msg); // contains distance
        void StopNodeCallback(const std_msgs::Bool::ConstPtr& msg); // stops the node

        void PublishLidarData();

        //TODO add callback for sonar and encoder
        void VelocityCalculation();

        void run();
        void ClearData();

    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subDistance;
        ros::Subscriber m_subStopNode; // added semicolon
        //TODO: add subscriber for sonar and encoder

        ros::Publisher m_pubLidarData;
        //TODO add publisher for sonar and encoder

        double m_beginTime; 
        double m_adjustTimeToStartAtZero;
        bool m_stopNode;
        std::string m_type;

        std::vector<double> m_time;
        std::vector<double> m_timeVelocityPlot;
        std::vector<float> m_distance;
        std::vector<float> m_velocity;
        DistanceOperations m_operationsDistance;

};