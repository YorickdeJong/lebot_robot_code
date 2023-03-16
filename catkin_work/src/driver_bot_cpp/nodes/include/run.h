#pragma once
#include <ros/ros.h>

#include "driver_bot_cpp/objectAngle.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "../../actions/include/actionsFactory.h"
#include "../../magicNumbers/magicNumbers.h"
#include "../../motorImplementation/include/MotorDetails.h"

class Drive
{
    public:
        Drive();
        ~Drive() = default;
        
        void ObjectAngleCallback(const driver_bot_cpp::objectAngle::ConstPtr& msg);
        void GammaCallback(const std_msgs::Float32::ConstPtr& msg);
        void DistToWallCallback(const std_msgs::Float32::ConstPtr& msg);
        void MinDistanceCallback(const std_msgs::Float32::ConstPtr& msg);

        bool ValidationCallbacks();
        void CheckCondition();
        void AdjustPosition();
        void Run();

        

    private:
        void WallLeftAdjustRight(const float error, const float thresholdDist, const int velocity, const float thresholdAngle);
        void WallLeftAdjustLeft(const float error, const float thresholdDist, const int velocity, const float thresholdAngle);
        void WallRightAdjustRight(const float error, const float thresholdDist, const int velocity, const float thresholdAngle);
        void WallRightAdjustLeft(const float error, const float thresholdDist, const int velocity, const float thresholdAngle);
        void WallRightAhead(const float thresholdDistance, const int velocity, const float thresholdAngle);

    private:
        ros::NodeHandle m_node;
        ros::Subscriber m_subObjectAngle;
        ros::Subscriber m_neirestDistance;
        ros::Subscriber m_subGamma;
        ros::Subscriber m_subDist;

        std::unique_ptr<ActionsInterface> m_actions;

        float m_gamma;
        float m_distToWall;
        float m_angle_new;
        float m_minDistance;
        
        bool m_rightSide;
        int m_sign;

};