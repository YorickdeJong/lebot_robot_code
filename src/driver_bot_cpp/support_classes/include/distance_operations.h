#pragma once
#include <vector>
#include <iostream>
#include <math.h>

#include "../../magicNumbers/magicNumbers.h"

class DistanceOperations
{
    public:
        DistanceOperations(const float angle1, const float angle2);
        ~DistanceOperations() = default;
        
        //Main functionalities
        std::vector<float> DataAnalyses();
        float Detection(const std::vector<float>& angleArray);

        //Getters
        float GetAngleIncrement(){return m_angleIncrement;};
        std::vector<float> GetDist(){return m_dist;};

        //Setters
        void SetAngleIncrement(float angleIncrement){m_angleIncrement = angleIncrement;}; 
        void SetDist(const std::vector<float>& distance){m_dist = distance;};//shared_ptr

        //Support functions
        float MinimumDistance(const std::vector<float>& distanceArray);
        float MinimumDistance(const std::vector<float>& (&distanceArray)());
        std::vector<int> FindAngleIndices(const float angle1, const float angle2, const std::vector<float>& angleArray);
        std::vector<float> GetIndices(std::vector<int>& idxAngle, const std::vector<float>& distance);
        float AngleWithAssociatedDistance(const float distance, const std::vector<float>& angleArray);
    
    private:
        float m_angleIncrement;
        float m_angle1;
        float m_angle2;
        std::vector<float> m_dist; //shared_ptr
};
