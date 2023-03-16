#pragma once
#include <opencv2/opencv.hpp>
#include <tuple>
#include <utility>
#include <vector>

class Draw
{
    public:
        Draw();
        ~Draw() = default;
        
        std::tuple<int, cv::Point2f> Circle(cv::Mat& mask, const cv::Mat& frame); 
        std::tuple<int, cv::Point2f> Contour(cv::Mat& mask, const cv::Mat& frame); 
};