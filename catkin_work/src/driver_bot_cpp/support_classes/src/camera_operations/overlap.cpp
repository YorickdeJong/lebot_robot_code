#include "overlap.h"


Overlap::Overlap()
:m_overlapping{false}, m_delay{ros::Time::now()}
{

}

bool Overlap::CircleInCircle(float radiusContour, float radiusCircle, cv::Point2f & centerContour, cv::Point2f & centerCircle)
{
    /* Checks if circle is inside of contour
    Args:
        radiusContour: radius of contour around target retrieved from Draw.contour()

        radiusCircle:  radius of circle around target retrieved from Draw.circle()

        centerContour: center of contour in target retrieved from Draw.contour()

        centerCircle: center of circle in target retrieved from Draw.circle()

    Returns:
        move command for the robot with a delay of x seconds such that the command
        cannot be executed again in this time frame
    */
    float dist_square = std::pow((centerContour.x - centerCircle.x), 2) + std::pow((centerContour.y - centerCircle.y), 2);
    
    ros::Time time = ros::Time::now();

    m_overlapping = false;

    if (std::pow((dist_square + radiusCircle), 2) < std::pow((radiusContour), 2) && m_delay <= time)
    {
        m_delay = ros::Time::now() + ros::Duration{5};
        m_overlapping = true;
        std::cout << "Overlap, Blue Circle Detected" << std::endl;
    }

    return m_overlapping; 
}