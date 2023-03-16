#include "distance_operations.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

DistanceOperations::DistanceOperations(const float angle1, const float angle2)
:m_angleIncrement{ANGLE_INCREMENT_START_VALUE}, m_dist{DISTANCE_START_VALUE},
m_angle1{angle1}, m_angle2{angle2}
{
}

std::vector<float> DistanceOperations::DataAnalyses()
{
    /* Produces an array of of values with increment self.angle_increment for one round
    trip between 0 and 2 pi.We reverse the list since the rpLidar scanner starts at 2pi
    and ends at 0 pi */

    //Eq to linspace in python
    std::vector<float> Array;
    int idxs = 2 * M_PI / m_angleIncrement;
    
    if (m_angleIncrement == 1)
    {
        Array = {DEFAULT_ARAY_VALUE};
        return Array;
    }
    for (int i{0}; i < idxs; i++)
    {
        float times2 = TWO_PI * i;
        float number = times2/idxs;
        Array.emplace_back(number);
    }
    return Array;
}
float DistanceOperations::Detection(const std::vector<float>& angleArray)
{

    /*Breaks up distance object from 0 to 2pi into a specific angle range
    Variables:
        angle1: lower bound of angle range
        angle2: upper bound of angle range

    Returns:
        array of distance values for angle1 < angle < angle2
    */

    std::vector<float> distanceRange;
    std::vector<int> distanceIdx = FindAngleIndices(m_angle1, m_angle2, angleArray);

    distanceRange = GetIndices(distanceIdx, m_dist);
    return MinimumDistance(distanceRange);


}


//Support functions
std::vector<int> DistanceOperations::FindAngleIndices(const float angle1, const float angle2, const std::vector<float>& angleArray)
{
    /*function finds at distance for specific angle range
    Args:
        angle1: lower bound angle
        angle2: upper bound angle
        angleArray: array containing angles for 0 to 2pi
    
    Returns: array with specific angle range 
    */

    std::vector<int> idxAngle; 
    for (int i{}; i < angleArray.size(); i++)
    {
        if(angleArray[i] < angle1 || angleArray[i] > angle2)
        {
            idxAngle.emplace_back(i); 
        }
    }
    return idxAngle;
}

std::vector<float> DistanceOperations::GetIndices(std::vector<int>& idxAngle, const std::vector<float>& distance)
{   
    /*Retrieves values from distance based on indices provided in idxAngle
    Args:
        idxAngle: array containing indices for specific angles

        distance: array containing distance values between 0 and 2pi
    Returns: Array with wanted values, based on elements provided in arrayIdx
    */
    std::vector<float> newDistArray;
    
    if (idxAngle.size() == 0 || distance.size() == 0)
    {
        newDistArray = {DISTANCE_START_VALUE};
        return newDistArray;
    }


    idxAngle.pop_back(); //removing element containing zero 
    newDistArray.reserve(idxAngle.size());

    for (const int& i : idxAngle)
    {
        newDistArray.emplace_back(distance[i]);
    }

    return newDistArray;

}

float DistanceOperations::MinimumDistance(const std::vector<float>& distanceArray)
{
    /*Function calculates minimum distance to object
    Args:
        distance: array containing distance to all objects between angle1 and angle2 for 0.15m < d < 12m

    Returns:
        minimum distance of the provided distance array
    */
    float temp {GREATER_THAN_MAXIMUM_DISTANCE};
    for (const float& dist: distanceArray)
    {
        if (temp > dist)
        {
            temp = dist;
        }
    }
    return temp;
}


float DistanceOperations::AngleWithAssociatedDistance(const float distance, const std::vector<float>& angleArray)
{
    if(angleArray.size() != m_dist.size())
    {
        std::cout << "[ERROR] ANGLE DISTANCE DID NOT MATCH DISTANCE IN ARRAY" << std::endl;
        return -1;
    }
    
    float angle;
    for (int i{}; i < m_dist.size(); i++)
    {
        if (fabsf(distance - m_dist[i]) < ALLOWABLE_ERROR_DISTANCE_DISTANCEARRAY)
        {
            angle = angleArray[i];
            return angle;
        }
    }
    std::cout << "[ERROR] ANGLE DISTANCE DID NOT MATCH DISTANCE IN ARRAY" << std::endl;
    return angle = -1;
}



