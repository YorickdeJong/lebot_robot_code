#pragma once
#include <ros/ros.h>

#include "std_msgs/Float32.h"

#include "../../support_classes/include/angle_operations.h"
#include "driver_bot_cpp/lidar.h"
#include "driver_bot_cpp/objectAngle.h"
#include "../../magicNumbers/magicNumbers.h"

class AngleDetection
{
    public:
        AngleDetection();
        ~AngleDetection() = default;

        //Main functionalities
        void LidarCallback(const driver_bot_cpp::lidar::ConstPtr &msg);
        void Run();

        //additional functionality
        void PubGammaData(float angleNew);
        void PubObjectLocation(bool rightSide, float angleNew, int sign);



    private:
        ros::NodeHandle m_node;
        ros::Publisher m_pubGamma;
        ros::Publisher m_pubObjectAngle;
        ros::Subscriber m_subLidar;

        AngleOperations m_operationsAngle;

};