#include "frame_analysis.h"

FrameAnalysis::FrameAnalysis()
{}

cv::Mat FrameAnalysis::HsvFilter(const cv::Mat& frame)
{
    /*Filters out bgr values
        Args:
            frame: frame provided by camera

        Returns: 
            hue, saturation, value of frame, more accurate than regular rgb
    */
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    return hsv;
} 

cv::Mat FrameAnalysis::MaskingBlueFilter(cv::Mat& hsv)
{
    /*Filters out blue values
        Args:
            hsv: hue, saturation, value

        Returns:
            image filtered on blue values
    */
    cv::Scalar lowerBlue(101, 50, 38);
    cv::Scalar upperBlue(110, 252, 255);
    cv::Mat mask;
    cv::inRange(hsv, lowerBlue, upperBlue, mask);
    return mask;
}

cv::Mat FrameAnalysis::BitWiseFilter(cv::Mat& mask, const cv::Mat& frame)
{
    /*converts values to black or white
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            converts each corresponding number in mask and frame to its binary form 
            and then returns either black or white pizxels 
    */
    cv::Mat target = mask;
    cv::bitwise_and(frame, frame, target);
    return target;
}

