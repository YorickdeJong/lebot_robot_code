#include "angle_operations.h"


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

AngleOperations::AngleOperations()
:m_dist{DISTANCE_START_VALUE}, m_angleIncrement{ANGLE_INCREMENT_START_VALUE}, m_angle{ANGLE_START_VALUE}
{
}

void AngleOperations::DataAnalyses()
{
    /*Calculates the angle at which the minimum distance to an object occurs*/
    float unit_circle = TWO_PI * M_PI;
    float increments_in_circle = unit_circle / m_angleIncrement;
    float angle = MinimumDistanceArg() / increments_in_circle * TWO_PI; //time 2 again since max minimumDistanceArg = max increment_in_circle
    m_angle = angle;
}


void AngleOperations::Detection()
{
   /*Check where the wall of the robot is and adjust the angle and
    *make sure that new angle is always smaller than 1/2 for gamma calc
    *Args:
    *   angle: angle at which minimum distance occurs
    *
    *Returns:
    *   m_rightSide: True if wall is on the right side of the vehicle
    *   m_angle_new: adjusted angle for position of the wall compared to the vehicle
    *   m_sign: contains direction of the vehicle's y velocity in world coordinates
    *   publishes data on 'objectPosition' topic
    */
    DataAnalyses();
    UpperLeftQuadrant();
    LowerLeftQuadrant();
    LowerRightQuadrant();
    UpperRightQuadrant();
    FrontView();
}


//support functions
float AngleOperations::MinimumDistanceArg()
{
    /*Calculates index for a minimum distance*/
    float temp{GREATER_THAN_MAXIMUM_DISTANCE};

    int idx;
    for (int i{}; i < m_dist.size(); i++)
    {
        if (temp > m_dist[i])
        {
            temp = m_dist[i];
            idx = i;
        } 
    }
    std::cout << idx << std::endl;
    return idx;
}

void AngleOperations::UpperLeftQuadrant()
{
    /*Checks if object is in the upper left quadrant of robot frame.
    if so, m_rightSide, m_angle_new, m_sign are set*/
    if (ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT < m_angle && m_angle < ANGLE_QUADRANT_ONE)
    {
        //moving towards left wall
        std::cout << "min 0" << std::endl;
        m_rightSide = false;
        m_angle_new = m_angle;
        m_sign = NEGATIVE_VELOCITY_DIRECTION;
    }
}

void AngleOperations::LowerLeftQuadrant()
{
    /*Checks if object is in the lower left quadrant of robot frame.
    if so, m_rightSide, m_angle_new, m_sign are set*/
    if (ANGLE_QUADRANT_ONE < m_angle && m_angle < ANGLE_QUADRANT_TWO)
    {
        //moving away from left wall
        std::cout << "min 0.5" << std::endl;
        m_rightSide = false;
        m_angle_new = m_angle - ANGLE_QUADRANT_ONE;
        m_sign = POSITIVE_VELOCITY_DIRECTION;
    }

}

void AngleOperations::LowerRightQuadrant()
{
    /*Checks if object is in the lower right quadrant of robot frame.
    if so, m_rightSide, m_angle_new, m_sign are set*/
    if (ANGLE_QUADRANT_TWO < m_angle && m_angle < ANGLE_QUADRANT_THREE)
    {
        //moving away from right wall
        std::cout << "min 1" << std::endl;
        m_angle_new = m_angle - ANGLE_QUADRANT_TWO;
        m_rightSide = true;
        m_sign = NEGATIVE_VELOCITY_DIRECTION;
    }
}

void AngleOperations::UpperRightQuadrant()
{
    /*Checks if object is in the upper right quadrant of robot frame.
    if so, m_rightSide, m_angle_new, m_sign are set*/
    if (ANGLE_QUADRANT_THREE < m_angle && m_angle < ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT)
    {
        //moving towards right wall
        std::cout << "min 1.5" << std::endl;
        m_angle_new = m_angle - ANGLE_QUADRANT_THREE;
        m_rightSide = true;
        m_sign = POSITIVE_VELOCITY_DIRECTION;
    }
}

void AngleOperations::FrontView()
{
    /*Checks if object is infront of the robot.
    if so, m_rightSide, m_angle_new, m_sign are set */
    if (m_angle <= ANGLE_ONE_DOMAIN_AT_FRONT_OF_ROBOT || m_angle >= ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT)
    {
        //Will not get triggered by wall
        //Prevents getting stuck
        if (m_angle >= ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT)
        {
            m_angle_new = m_angle - ANGLE_TWO_DOMAIN_AT_FRONT_OF_ROBOT;
        }
        else
        {
            m_angle_new = m_angle;
        }
        m_rightSide = false;
        m_sign = 0;
    }
    
}

