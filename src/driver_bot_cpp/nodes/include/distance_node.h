#pragma once
#include <ros/ros.h>
#include <vector>
#include "driver_bot_cpp/lidar.h"
#include "std_msgs/Float32.h"
#include "../../support_classes/include/distance_operations.h"
#include "../../magicNumbers/magicNumbers.h"

class DistanceDetection
{
    public:
        DistanceDetection(const float angle1, const float angle2);
        ~DistanceDetection() = default;
        
        //Main functionalities
        void LidarCallback(const driver_bot_cpp::lidar::ConstPtr& msg);
        void Run();

        //Additional functionalities
        void PublishDistanceRange(const float distance);
        void PublishMinDistance(const float distance);

    
    private:
        ros::NodeHandle m_node;
        ros::Publisher m_pubDistance;
        ros::Publisher m_pubMinDistance;
        ros::Subscriber m_subLidar;
        DistanceOperations m_operationsDistance;

};
