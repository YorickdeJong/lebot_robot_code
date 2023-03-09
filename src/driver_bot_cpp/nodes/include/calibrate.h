#pragma once
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <chrono>
#include <math.h>  
#include <tuple> 

#include "driver_bot_cpp/lidar.h"
#include "driver_bot_cpp/distanceVelocity.h"
#include "../../actions/include/actionsFactory.h"
#include "../../support_classes/include/distance_operations.h"
#include "../../support_classes/include/angle_operations.h"
#include "../../magicNumbers/magicNumbers.h"

class Calibrate
{   
    /*Calibration needs lidar node to be active in order to work*/
    public:
        Calibrate(const float angle1, const float angle2);
        ~Calibrate() = default; 
        
        //Main functions
        void DistanceCallback(const driver_bot_cpp::lidar::ConstPtr& msg);
        bool Validation();
        bool Calibration(const int calibrationSteps);
        void PublishDetailedVelocityData(std::vector<float>& distanceCalibration, std::vector<double>& timeCalibration);

        
    private:
        //Support Functions
        void CheckDistance(const float beginningThresholdMeter);
        void CountDown();
        void VelocityCalculation(const float thresholdMeter, const int initialSpeed, const int calibrationStep);
        void DriveToInitialPosition(const float beginningThresholdMeter);
        std::vector<float> Velocity(const std::vector<float>& distanceCalibration, const std::vector<double>& timeCalibration);
        void SaveDistanceTimePoints(double time);
        void ClearData();

    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subDistance;
        ros::Publisher m_pubDetailedVelocityResults;
        
        float m_angleIncrement; 
        float m_minDistanceRange;
        float m_angleAtDistance;
        float m_angle1;
        float m_angle2;
        float m_motorSpeed;
        
        double time;

        std::vector<float> m_pointDistance; //shared_ptr
        std::vector<double> m_pointTime; //shared_ptr

        std::unique_ptr<ActionsInterface> m_actions;
        DistanceOperations m_operationsDistance;
};