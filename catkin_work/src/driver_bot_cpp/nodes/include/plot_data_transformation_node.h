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
#include "driver_bot_cpp/distanceData.h"
#include <std_msgs/Float32MultiArray.h>

class DataTransformationPlot{

    public:
        DataTransformationPlot();
        ~DataTransformationPlot() = default;

        void DistanceCallback(const driver_bot_cpp::distanceData::ConstPtr& msg); // contains distance
        void StopNodeCallback(const std_msgs::Bool::ConstPtr& msg); // stops the node
        std::vector<std_msgs::Float32MultiArray> filterAndConvertVector(std::vector<std::vector<float>>& data);
        
        void PublishData();

        //TODO add callback for sonar and encoder
        void VelocityCalculation();
        void ForceCalculation();
        void EnergyCalculation();
        
        void run();
        void ClearData();

    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subDistance;
        ros::Subscriber m_subStopNode; // added semicolon
        //TODO: add subscriber for sonar and encoder

        ros::Publisher m_pubData;
        ros::Publisher m_pubEncoderData;
        //TODO add publisher for sonar and encoder

        double m_beginTime; 
        double m_adjustTimeToStartAtZero;
        bool m_stopNode;
        bool m_setMotors;
        std::string m_type;

        //need 4 times velocity, distance, force and energy
        std::vector<double> m_time;
        std::vector<int> m_motorNumber;

        std::vector<std::vector<float>> m_distance;
        std::vector<std::vector<float>> m_velocity;
        std::vector<std::vector<float>> m_force;
        std::vector<std::vector<float>> m_energy;
        
        DistanceOperations m_operationsDistance;
        int m_rate;
};