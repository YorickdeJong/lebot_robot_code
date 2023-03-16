#pragma once
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ros/ros.h>
#include <cmath>

class Overlap
{
    public:
        Overlap();
        ~Overlap() = default; 
        bool CircleInCircle(float radiusContour, float radiusCircle, cv::Point2f & centerContour, cv::Point2f & centerCircle);
    
    private:
        bool m_overlapping;
        ros::Time m_delay;

};