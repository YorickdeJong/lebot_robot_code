#pragma once
#include <vector>
#include <tuple>
#include <iostream>

#include "../../magicNumbers/magicNumbers.h"

class AngleOperations
{
    public:
        AngleOperations();
        ~AngleOperations() = default;
        
        //Main functionalities
        void DataAnalyses();
        void Detection();

        //setters
        void SetAngleIncrement(const float angleIncrement){m_angleIncrement = angleIncrement;};
        void SetDist(const std::vector<float>& distance){m_dist = distance;};
        
        //getters
        bool GetRightSide(){return m_rightSide;};
        float GetAngleNew(){return m_angle_new;};
        int GetSign(){return m_sign;};


        //support functions
        float MinimumDistanceArg();
        void UpperLeftQuadrant();
        void LowerLeftQuadrant();
        void LowerRightQuadrant();
        void UpperRightQuadrant();
        void FrontView();
        


    private:
        std::vector<float> m_dist; //shared_ptr
        float m_angleIncrement; 
        float m_angle;
        
        bool m_rightSide;
        float m_angle_new;
        int m_sign;
};