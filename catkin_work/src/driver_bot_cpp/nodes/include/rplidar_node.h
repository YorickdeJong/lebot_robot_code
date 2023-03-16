#pragma once
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "driver_bot_cpp/lidar.h"
#include "std_msgs/Bool.h"
#include <vector>
#include <iostream>
#include <memory>

#include "std_msgs/String.h"
#include "../../magicNumbers/magicNumbers.h"

class RPLidar
{
    public:
        RPLidar();
        ~RPLidar() = default;
        
        //General Sensor Functions
        void Read(const sensor_msgs::LaserScan::ConstPtr &msg);
        void Validate();
        void Run();

        //Support Functions
        void KeyStrokesCallback(const std_msgs::String::ConstPtr &msg);
        void DistanceCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void AngleCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void PublishValidatedData();
        void PublishStopNode();


        
    private:
    //init node
        ros::NodeHandle m_node; //create a node handle

    //defining variables
        std::vector<float> m_dist; //make reference as we do not wish to copy the data from msg
        double m_angleIncrement;
        ros::Subscriber m_subLidar;
        ros::Subscriber m_subKeyStrokes;
        ros::Publisher m_pubValData;
        ros::Publisher m_pubStopNode;
        bool m_stopNode;
        std::string m_key;
};
