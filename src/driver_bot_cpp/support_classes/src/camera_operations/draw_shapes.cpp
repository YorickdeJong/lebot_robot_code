#include "draw_shapes.h"

Draw::Draw()
{}

std::tuple<int, cv::Point2f> Draw::Circle(cv::Mat& mask, const cv::Mat& frame)
{
    /*Draws circle on objects with a certain color
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            radius and center of circle around the target object
    */
    cv::Moments M = cv::moments(mask);
    int radius;
    
    if (M.m00 > 0)
    {
        int cx = M.m10/M.m00;
        int cy = M.m01/M.m00;
        radius = 30;
        cv::Point2f center(cx, cy);
        cv::circle(frame, center, radius, (0, 0, 255), -1);
        return std::make_tuple(radius, center);
    }
}


std::tuple<int, cv::Point2f> Draw::Contour(cv::Mat& mask, const cv::Mat& frame)
{
    /* Contour of image objects
        Args:
            mask: image filtered on blue values

            frame: unaltered frame image
        Returns:
            radius and center of contour circle around the target object 
    */
    
    // get contour around object

    std::vector<std::vector<cv::Point>> contour; 
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contour, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    // define biggest contour and its corresponding index
    float maxCountour = -1, tempContour;
    int idxMax = 0;
    float radius;

    cv::Point2f center;
    std::vector<cv::Point> contourMax;

    // find biggest contour and its corresponding index
    for (int i{}; i < contour.size(); i++)
    {
        tempContour = cv::contourArea(contour[i]);
        if (maxCountour < tempContour)
        {
            maxCountour = tempContour;
            idxMax = i;
        }
    }

    // find enclosing circle
    contourMax = contour[idxMax];
    
    cv::minEnclosingCircle(contourMax, center, radius);

    if (radius > 40)
    {
        radius = 40;
    }

    //draw contour
    cv::circle(frame, center, radius, (0, 255, 0), 2);
    return std::make_tuple(radius, center);

}