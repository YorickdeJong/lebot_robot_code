#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream> // for converting the command line parameter to integer
#include <stdexcept>
#include <string>

class FrameAnalysis
{
    public:
        FrameAnalysis();
        ~FrameAnalysis() = default;
        cv::Mat HsvFilter(const cv::Mat& frame);
        cv::Mat MaskingBlueFilter(cv::Mat& hsv);
        cv::Mat BitWiseFilter(cv::Mat& mask, const cv::Mat& frame);
};