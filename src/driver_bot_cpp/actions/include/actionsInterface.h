#pragma once 
#include "../../motorImplementation/include/motorFactory.h"
#include "../../magicNumbers/magicNumbers.h"

class ActionsInterface
{
    public:
        virtual ~ActionsInterface() = default;

        virtual void Stop() = 0;
        
        virtual void DriveForward(const int velocity) = 0;
        virtual void DriveBackward(const int velocity) = 0;
        
        virtual void TurnRight(const int velocity) = 0;
        virtual void TurnLeft(const int velocity) = 0;

        virtual bool Turn90DegreesRight(const float gamma, const int velocity) = 0;
        virtual bool Turn90DegreesLeft(const float gamma, const int velocity) = 0;

        virtual bool CorrectForDistance(const float distToWall, const int velocity, float threshold) = 0;
        virtual bool DriveToWall(const float distToWall, const int velocity, float threshold) = 0;
        virtual bool CorrectForAngleBetweenWorldAndRobotFrame(const float gamma, const int sign, const int velocity, float thresholdAngle) = 0;
        


        virtual std::unique_ptr<ActionsInterface> Instantiate() = 0;
};

