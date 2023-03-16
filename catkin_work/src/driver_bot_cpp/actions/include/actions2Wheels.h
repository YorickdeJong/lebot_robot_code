#pragma once 
#include "actionsInterface.h"

class Actions2Wheels : public ActionsInterface
{
    public:
        Actions2Wheels();
        ~Actions2Wheels() = default;
        void Stop() override;
        
        void DriveForward(const int velocity) override;
        void DriveBackward(const int velocity) override;
        
        void TurnRight(const int velocity) override;
        void TurnLeft(const int velocity) override;

        bool Turn90DegreesRight(const float gamma, const int velocity) override;
        bool Turn90DegreesLeft(const float gamma, const int velocity) override;

        bool CorrectForDistance(const float distToWall, const int velocity, float threshold) override;
        bool DriveToWall(const float distToWall, const int velocity, float threshold) override;
        bool CorrectForAngleBetweenWorldAndRobotFrame(const float gamma, const int sign, const int velocity, float thresholdAngle) override;

        std::unique_ptr<ActionsInterface> Instantiate() override;
        

        
    private:
        std::unique_ptr<MotorInterface> m_motor; 
        

};

